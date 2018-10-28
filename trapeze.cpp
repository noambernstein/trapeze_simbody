#include "Simbody.h"
#include <cmath>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <map>
#include <fstream>
using namespace SimTK;

void read_map(std::string filename, std::map<std::string,double> &param_map) {
    std::ifstream inFile;
    inFile.open(filename);

    std::string key, value;
    while (inFile >> key >> value) {
        param_map[key] = std::stod(value);
    }

    inFile.close();
}

#define USAGE "Usage: "<<argv[0]<<" [ --slow RATE ] [ --rig file ] [ --flyer file ] [ --initial_state file ] [ --poses file ]"

void args_to_map(int argc, char *argv[], std::map<std::string,std::string> &args_map) {
    for (int i=1; i < argc; i++) {
        if (i < argc-1) {
            args_map[std::string(argv[i])] = std::string(argv[i+1]);
            i += 1;
        } else {
            std::cerr << "Got argument number "<<i<<" '"<<argv[i]<<"' but no following value" << std::endl;
            std::cerr << USAGE << std::endl;
            exit(1);
        }
    }
}

void parse_args(int argc, char *argv[], double *slow_mo_rate, double *gravity_accel,
    std::string *rig_filename, std::string *flyer_filename, std::string *initial_state_filename, std::string *poses_filename) {

    std::map<std::string,std::string> args_map;

    args_to_map(argc, argv, args_map);

    if (args_map.find("--slow") == args_map.end()) {
        *slow_mo_rate = 1.0;
    } else {
        *slow_mo_rate = 1.0/std::stod(args_map["--slow"]);
    }
    if (args_map.find("--gravity") == args_map.end()) {
        *gravity_accel = 9.8;
    } else {
        *gravity_accel = std::stod(args_map["--gravity"]);
    }
    if (args_map.find("--rig") == args_map.end()) {
        *rig_filename = "rig.data";
    } else {
        *rig_filename = args_map["--rig"];
    }
    if (args_map.find("--flyer") == args_map.end()) {
        *flyer_filename = "flyer.data";
    } else {
        *flyer_filename = args_map["--flyer"];
    }
    if (args_map.find("--initial_state") == args_map.end()) {
        *initial_state_filename = "initial_state_board.data";
    } else {
        *initial_state_filename = args_map["--initial_state"];
    }
    if (args_map.find("--poses") == args_map.end()) {
        *poses_filename = "poses.data";
    } else {
        *poses_filename = args_map["--poses"];
    }

}

void read_sys_state(std::string filename, std::map<std::string,int> joint_index, 
    double &lines_pos, double &lines_vel, std::map<std::string,double> &joint_pos, std::map<std::string,double> &joint_vel,
    std::string &pose_name) {
    std::ifstream inFile;
    std::string key, pos_value, vel_value;

    std::cout << "reading initial state from "<<filename<<std::endl;

    lines_pos = 0;
    lines_vel = 0;
    for (auto p : joint_index) {
        joint_pos[p.first] = 0.0;
        joint_vel[p.first] = 0.0;
    }

    inFile.open(filename);
    while (inFile >> key) { 
        std::cerr << "  reading key " << key << std::endl;
        if (key == "POSE") { // initial pos
            inFile >> pose_name;
        } else { // joint pos vel
            inFile >> pos_value >> vel_value;
            if (key == "lines") {
                lines_pos = std::stod(pos_value)*M_PI/180.0;
                lines_vel = std::stod(vel_value)*M_PI/180.0;
            } else {
                if (joint_index.find(key) == joint_index.end()) {
                    std::cerr << "system state key "<<key<<" not in joint_index" << std::endl;
                    exit(1);
                }
                joint_pos[key] = std::stod(pos_value)*M_PI/180.0;
                joint_vel[key] = std::stod(vel_value)*M_PI/180.0;
            }
        }
    }
    inFile.close();

}

class Joints {
public:
    Joints () {}

    Joints (std::map<std::string, std::pair<MobilizedBody *,double>> joint_map) {
        int i = 0;
        for (auto p : joint_map) {
            index[p.first] = i;
            names.push_back(p.first);
            joints.push_back(p.second.first);
            max_torque.push_back(p.second.second);
            i++;
        }
    }

    int size() {
        return joints.size();
    }

    MobilizedBody::Pin *get_pin(std::string name) {
        return get_pin(index[name]);
    }

    MobilizedBody::Pin *get_pin(int i) {
        return static_cast<MobilizedBody::Pin *>(joints[i]);
    }

    std::vector<MobilizedBody *> joints;
    std::vector<double> max_torque;
    std::vector<std::string> names;
    std::map<std::string, int> index;
};

void set_sys_state(State &state, double lines_pos, double lines_vel, std::map<std::string,double> &pos, std::map<std::string,double> &vel, 
    MobilizedBody *lines, Joints &joints) {

    static_cast<MobilizedBody::Pin *>(lines)->setAngle(state, lines_pos);
    static_cast<MobilizedBody::Pin *>(lines)->setRate(state, lines_vel);
    for (auto p : pos) {
        joints.get_pin(p.first)->setAngle(state, pos[p.first]);
    }
    for (auto p : vel) {
        joints.get_pin(p.first)->setRate(state, vel[p.first]);
    }
}

#define INF_ANGLE 1.0e6

class Pose {
public:
    Pose(std::string filename, std::map<std::string, double> flyer_params, Joints &joints,  GeneralForceSubsystem &forces) :
        n_joints(joints.size()), cur_pose(NULL), l_joints(&joints) {

        reached_pose.reserve(n_joints);
        prev_delta_angle.reserve(n_joints);
        for (int i = 0; i < n_joints; i++) {
            reached_pose[i] = 0;
            prev_delta_angle[i] = 0.0;
        }

        std::cout << "reading poses from "<<filename<<std::endl;
        std::ifstream inFile;

        inFile.open(filename);

        std::string pose_label, pose_name, base_pose, joint_name;
        double joint_angle;
        std::string applied_torque;
        int n_joints_set;
        char key;
        while (inFile >> pose_label >> pose_name >> key >> n_joints_set >> base_pose) {
            if (pose_label != "POSE") {
                std::cerr << "unknown label '" << pose_label << "' in poses file " << filename << std::endl;
                std::exit(1);
            }

            if (keymap.find(key) != keymap.end()) {
                std::cerr << "got key '"<<key<<"' twice, first for '"<<keymap[key]<<"' then for '"<<pose_name<<"'"<<std::endl;
                exit(1);
            }
            keymap[key] = pose_name;

            // allocate and create local reference;
            poses[pose_name];
            std::vector<double> & pose = poses[pose_name];
            pose.reserve(2*n_joints);

            // never any control over anchor or hands"
            pose[joints.index["lines_anchor"]*2+0] = 0.0;
            pose[joints.index["lines_anchor"]*2+1] = 0.0;
            pose[joints.index["hands"]*2+0] = 0.0;
            pose[joints.index["hands"]*2+1] = 0.0;
            // copy base pose
            if (base_pose != "-") {
                for (int i = 0; i < n_joints*2; i++) {
                    poses[pose_name][i] = poses[base_pose][i];
                }
            } else if (n_joints_set != n_joints) {
                std::cerr << "base_pose not set, n_joints_set = " << n_joints_set << " must equal n_joints - 2 = " << (n_joints) << std::endl;
                exit(1);
            }
            // set current pose
            for (int i = 0; i < n_joints_set; i++) {
                inFile >> joint_name >> joint_angle >> applied_torque;
                int joint_i = joints.index[joint_name];
                pose[joint_i*2+0] = joint_angle * M_PI / 180.0;
                if (applied_torque == "-") {
                    pose[joint_i*2+1] = -1.0;
                } else {
                    pose[joint_i*2+1] = std::stod(applied_torque);
                }
            }
        }

        inFile.close();

        // create inactive actuators for pose hold (very large bounds) and pose torque (zero torque)
        for (int i = 0; i < n_joints; i++) {
            joint_hold_actuators.push_back(SimTK::Force::MobilityLinearStop::MobilityLinearStop(forces, *(l_joints->get_pin(i)), MobilizerQIndex(0), 
                flyer_params["joint_hold_stiffness"], flyer_params["joint_hold_damping"], -INF_ANGLE, INF_ANGLE));
            joint_torque_actuators.push_back(SimTK::Force::MobilityConstantForce::MobilityConstantForce(forces, *(l_joints->get_pin(i)), 0.0));
        }
    }

    bool do_keypress(unsigned key) {
        if (keymap.find(key) != keymap.end()) {
            std::cerr << "do_keypress doing set_pose "<<keymap[key]<<std::endl;
            set_pose(keymap[key]);
            return true;
        } else {
            return false;
        }
    }

    void set_pose(std::string pose_name) {
        cur_pose = &(poses[pose_name]);
        for (int i=0; i < n_joints; i++) {
            reached_pose[i] = 0;
            prev_delta_angle[i] = 0.0;
        }
    }

    void set_forces(State &state) {
        if (cur_pose == NULL) {
            return;
        }
        for (int i=0; i < n_joints; i++) {
            if (!reached_pose[i]) {
                double pose_angle = (*cur_pose)[2*i+0];
                double cur_delta_angle = l_joints->get_pin(i)->getAngle(state) - pose_angle;
                if (std::abs(cur_delta_angle) < 0.001 || cur_delta_angle*prev_delta_angle[i] < 0.0) {
                    // just reached (or passed) pose, disable torque, enable hold
                    reached_pose[i] = 1;
                    joint_torque_actuators[i].setForce(state, 0.0);
                    joint_hold_actuators[i].setBounds(state, pose_angle-0.001, pose_angle+0.001);
                    std::cerr << "set_forces reached pose for joint "<<l_joints->names[i]<<" disable torque enable hold\n";
                } else if (prev_delta_angle[i] == 0.0) {
                    // first time for this pose, disable hold, enable torque
                    joint_hold_actuators[i].setBounds(state, -INF_ANGLE, INF_ANGLE);
                    double applied_torque;
                    if ((*cur_pose)[2*i+1] >= 0.0) {
                        applied_torque = (*cur_pose)[2*i+1];
                    } else {
                        applied_torque = l_joints->max_torque[i];
                    }
                    joint_torque_actuators[i].setForce(state, applied_torque*(cur_delta_angle < 0.0 ? 1.0 : -1.0));
                    std::cerr << "set_forces first time trying pose for joint "<<l_joints->names[i]<<" disable hold enable torque "<< applied_torque<<std::endl;
                }
                prev_delta_angle[i] = cur_delta_angle;
            }
        }
    }

private:
    int n_joints; // array size will be n_joints*2;
    std::vector<double> *cur_pose;
    std::vector<int> reached_pose;
    std::vector<double> prev_delta_angle;
    std::vector<SimTK::Force::MobilityLinearStop::MobilityLinearStop> joint_hold_actuators;
    std::vector<SimTK::Force::MobilityConstantForce::MobilityConstantForce> joint_torque_actuators;
    std::map<std::string,std::vector<double> > poses;
    Joints *l_joints;
    std::map<unsigned,std::string> keymap;
};

// This is a custom InputListener. We'll register it prior to the InputSilo so
// that we can intercept all input and say something about it. No input processing
// is done here other than that, and we pass on everything we receive down the
// chain to the next listener (which will be an InputSilo in this case).
class MyListener : public Visualizer::InputListener {
public:
    MyListener(Pose *poses) : l_poses(poses) {}

    ~MyListener() {}

    virtual bool keyPressed(unsigned key, unsigned modifier) override {
        // String mod;
        // if (modifier&ControlIsDown) mod += "CTRL ";
        // if (modifier&ShiftIsDown) mod += "SHIFT ";
        // if (modifier&AltIsDown)  mod += "ALT ";

        return (l_poses->do_keypress(key));

        /*
        const char* nm = "NoNickname";
        switch(key) {
        case KeyEsc: nm="ESC"; break;
        case KeyDelete: nm="DEL"; break;
        case KeyRightArrow: nm="Right"; break;
        case KeyLeftArrow: nm="Left"; break;
        case KeyUpArrow: nm="Up"; break;
        case KeyDownArrow: nm="Down"; break;
        case KeyEnter: nm="ENTER"; break;
        case KeyF1: nm="F1"; break;
        case KeyF12: nm="F12"; break;
        case 'a': nm="lower a"; break;
        case 'Z': nm="upper Z"; break;
        case '}': nm="right brace"; break;
        }
        if (modifier&IsSpecialKey)
            std::cout << "Listener saw special key hit: " 
                << mod << " key=" << key << " glut=" << (key & ~SpecialKeyOffset);
        else
            std::cout << "Listener saw ordinary key hit: " 
                << mod << char(key) << " (" << (int)key << ")";
        std::cout << " " << nm << std::endl;
        */

        return false; // key passed on
    }

    virtual bool sliderMoved(int whichSlider, Real value) override {
        printf("Listener sees slider %d now at %g\n", whichSlider, value);
        return false;   // slider move passed on
    }

private:
    Pose *l_poses;

};

class PoseForceUpdater : public PeriodicEventHandler {
public:
    PoseForceUpdater(Pose *pose, Real interval) : PeriodicEventHandler(interval), l_pose(pose) {}

    virtual void handleEvent(State &state, Real accuracy, bool &shouldTerminate) const override
    {
        l_pose->set_forces(state);
    }
private:
    Pose *l_pose;
};

// This is a periodic event handler that interrupts the simulation on a regular
// basis to poll the InputSilo for user input. If there has been some, process it.
class UserInputHandler : public PeriodicEventHandler {
public:
    UserInputHandler(Visualizer& viz,
                     Visualizer::InputSilo& silo, 
                     Real interval) 
    :   PeriodicEventHandler(interval), m_viz(viz), m_silo(silo) {}

    virtual void handleEvent(State& state, Real accuracy,
                             bool& shouldTerminate) const override 
    {
        while (m_silo.isAnyUserInput()) {
            unsigned key, modifiers;

            while (m_silo.takeKeyHit(key,modifiers)) {
                if (key == Visualizer::InputListener::KeyEsc) {
                    printf("User hit ESC!!\n");
                    shouldTerminate = true;
                    m_silo.clear();
                    return;
                }
                printf("Handler sees key=%u, modifiers=%u\n",key,modifiers);
            }

        }  
    }

private:
    Visualizer&            m_viz;
    Visualizer::InputSilo& m_silo;
};

typedef struct {
    Body::Rigid fly_crane, lines_bar, net;
    MobilizedBody fly_crane_anchor, net_anchor, lines_mobile;
    SimTK::Force::MobilityLinearDamper lines_fly_crane_damper;
} Rig;

void create_rig(MobilizedBody &ground, std::map<std::string, double> &rig_params, GeneralForceSubsystem &forces, Rig &rig) {
    rig.fly_crane = Body::Rigid(MassProperties(1.0, Vec3(0.0), Inertia(1.0)));
    rig.fly_crane.addDecoration(Transform(), DecorativeBrick(Vec3(rig_params["fly_crane_thickness"]/2.0,
        rig_params["fly_crane_thickness"]/2.0,rig_params["fly_crane_width"])));
    rig.fly_crane_anchor = MobilizedBody::Pin(ground, Transform(Vec3(0.0, rig_params["fly_crane_height"]+rig_params["fly_crane_thickness"]/2.0, 0.0)),
        rig.fly_crane, Transform(Vec3(0, 0, 0)));

    PolygonalMesh net_mesh;
    net_mesh.loadFile("net.obj");
    rig.net = Body::Rigid(MassProperties(1.0, Vec3(0.0), Inertia(1.0)));
    DecorativeMesh net_mesh_decor(net_mesh);
    net_mesh_decor.setRepresentation(DecorativeGeometry::DrawWireframe);
    net_mesh_decor.setLineThickness(2.0);
    net_mesh_decor.setColor(Vec3(1.0, 1.0, 1.0));
    rig.net.addDecoration(Transform(), net_mesh_decor);
    rig.net_anchor = MobilizedBody::Pin(ground, Transform(Vec3(rig_params["net_offset"], rig_params["net_height"], 0.0)),
        rig.net, Transform(Vec3(0, 0, 0)));

    // mass distributed lines + point mass bar
    double lines_bar_com = -(rig_params["lines_mass"] * rig_params["lines_length"]/2.0 + 
                             rig_params["bar_mass"]*rig_params["lines_length"])/(rig_params["lines_mass"]+rig_params["bar_mass"]);
    double lines_bar_inertia = (rig_params["lines_mass"]/rig_params["lines_length"]) * pow(rig_params["lines_length"],3)/3.0 + 
                                rig_params["bar_mass"] * pow(rig_params["lines_length"],3)/3.0;
    rig.lines_bar = Body::Rigid(MassProperties(rig_params["lines_mass"]+rig_params["bar_mass"], Vec3(0.0, lines_bar_com, 0.0), 
        Inertia(lines_bar_inertia)));
    rig.lines_bar.addDecoration(Transform(), DecorativeLine(Vec3(0.0, 0.0, -rig_params["bar_length"]/2.0), 
        Vec3(0.0, -rig_params["lines_length"], -rig_params["bar_length"]/2.0)));
    rig.lines_bar.addDecoration(Transform(), DecorativeLine(Vec3(0.0, 0.0,  rig_params["bar_length"]/2.0), 
        Vec3(0.0, -rig_params["lines_length"],  rig_params["bar_length"]/2.0)));
    rig.lines_bar.addDecoration(Transform(Rotation(90.0*M_PI/180.0,CoordinateAxis(2)),Vec3(0.0, -rig_params["lines_length"], 0.0)), 
                                 DecorativeCylinder(rig_params["bar_radius"], rig_params["bar_length"]/2.0));

    rig.lines_mobile = MobilizedBody::Pin(ground, Transform(Vec3(0.0, rig_params["fly_crane_height"], 0.0)),
        rig.lines_bar, Transform(Vec3(0, 0, 0)));

    rig.lines_fly_crane_damper = SimTK::Force::MobilityLinearDamper::MobilityLinearDamper(forces, rig.lines_mobile, MobilizerUIndex(0), 100.0);
}

typedef struct {
    Body::Rigid lower_arms, upper_arms, torso_head, upper_legs, lower_legs;
    MobilizedBody hands, elbows, shoulders, hips, knees;
    double hands_max_torque, elbows_max_torque, shoulders_max_torque, hips_max_torque, knees_max_torque;
    SimTK::Force::MobilityLinearDamper hands_damper, elbows_damper, shoulders_damper, hips_damper, knees_damper;
    SimTK::Force::MobilityLinearStop hands_range_limit, elbows_range_limit, shoulders_range_limit, hips_range_limit, knees_range_limit;
} Flyer;

void create_flyer(MobilizedBody lines_anchor, double lines_length, std::map<std::string, double> &flyer_params,
    GeneralForceSubsystem &forces, Joints &joints, Flyer &flyer) {
    // mass distributed lower arms
    double lower_arms_mass = 2.0 * flyer_params["body_density"] * flyer_params["lower_arms_length"] * 
        M_PI * flyer_params["lower_arms_radius"] * flyer_params["lower_arms_radius"];
    double lower_arms_com = -flyer_params["lower_arms_length"] / 2.0;
    double lower_arms_inertia = lower_arms_mass / flyer_params["lower_arms_length"] * pow(flyer_params["lower_arms_length"],3)/3.0;
    flyer.lower_arms = Body::Rigid(MassProperties(lower_arms_mass, Vec3(0.0, lower_arms_com, 0.0), Inertia(lower_arms_inertia)));
    flyer.lower_arms.addDecoration(Transform(Vec3(0.0,-flyer_params["lower_arms_length"]/2.0, -flyer_params["torso_width"]/2.0-flyer_params["upper_arms_radius"])), 
                                  DecorativeCylinder(flyer_params["lower_arms_radius"], flyer_params["lower_arms_length"]/2.0));
    flyer.lower_arms.addDecoration(Transform(Vec3(0.0,-flyer_params["lower_arms_length"]/2.0, flyer_params["torso_width"]/2.0+flyer_params["upper_arms_radius"])), 
                                  DecorativeCylinder(flyer_params["lower_arms_radius"], flyer_params["lower_arms_length"]/2.0));

    // mass distributed upper arms
    double upper_arms_mass = 2.0 * flyer_params["body_density"] * flyer_params["upper_arms_length"] * 
        M_PI * flyer_params["upper_arms_radius"] * flyer_params["upper_arms_radius"];
    double upper_arms_com = -flyer_params["upper_arms_length"] / 2.0;
    double upper_arms_inertia = upper_arms_mass / flyer_params["upper_arms_length"] * pow(flyer_params["upper_arms_length"],3)/3.0;
    flyer.upper_arms = Body::Rigid(MassProperties(upper_arms_mass, Vec3(0.0, upper_arms_com, 0.0), Inertia(upper_arms_inertia)));
    flyer.upper_arms.addDecoration(Transform(Vec3(0.0,-flyer_params["upper_arms_length"]/2.0, -flyer_params["torso_width"]/2.0-flyer_params["upper_arms_radius"])), 
                                  DecorativeCylinder(flyer_params["upper_arms_radius"], flyer_params["upper_arms_length"]/2.0));
    flyer.upper_arms.addDecoration(Transform(Vec3(0.0,-flyer_params["upper_arms_length"]/2.0, flyer_params["torso_width"]/2.0+flyer_params["upper_arms_radius"])), 
                                  DecorativeCylinder(flyer_params["upper_arms_radius"], flyer_params["upper_arms_length"]/2.0));

    // mass distributed torso+head
    double torso_mass = flyer_params["body_density"] * flyer_params["torso_length"] * flyer_params["torso_width"] * flyer_params["torso_depth"];
    double head_mass = flyer_params["body_density"] * (4.0/3.0) * M_PI * pow(flyer_params["head_radius"],3);
    // relative to point flyer_params["upper_arms_radius"] below top of torso
    double torso_head_com = (-torso_mass * (flyer_params["torso_length"]/2.0-flyer_params["upper_arms_radius"]) + 
        head_mass * (flyer_params["upper_arms_radius"]+flyer_params["neck_length"]+flyer_params["head_radius"])) / (torso_mass + head_mass);
    double torso_inertia_com = (1.0/12.0) * torso_mass * (flyer_params["torso_length"]*flyer_params["torso_length"] + flyer_params["torso_depth"]*flyer_params["torso_depth"]);
    double head_inertia_com = (2.0/5.0) * head_mass * flyer_params["head_radius"]*flyer_params["head_radius"];
    double torso_head_inertia = torso_inertia_com + torso_mass*pow(flyer_params["torso_length"]/2.0-flyer_params["upper_arms_radius"],3) +
                                head_inertia_com + head_mass*pow(flyer_params["upper_arms_radius"]+flyer_params["neck_length"]+flyer_params["head_radius"],2);
    flyer.torso_head = Body::Rigid(MassProperties(torso_mass+head_mass, Vec3(0.0, torso_head_com, 0.0), Inertia(torso_head_inertia)));
    flyer.torso_head.addDecoration(Transform(Vec3(0.0,-flyer_params["torso_length"]/2.0+flyer_params["upper_arms_radius"], 0.0)), 
        DecorativeBrick(Vec3(flyer_params["torso_depth"]/2.0, flyer_params["torso_length"]/2.0, flyer_params["torso_width"]/2.0)));
    flyer.torso_head.addDecoration(Transform(Vec3(0.0,flyer_params["neck_length"]+flyer_params["head_radius"]+flyer_params["upper_arms_radius"], 0.0)), 
        DecorativeSphere(flyer_params["head_radius"]));

    // mass distributed upper legs
    double upper_legs_mass = 2.0 * flyer_params["body_density"] * flyer_params["upper_legs_length"] * 
        M_PI * flyer_params["upper_legs_radius"] * flyer_params["upper_legs_radius"];
    double upper_legs_com = -flyer_params["upper_legs_length"] / 2.0;
    double upper_legs_inertia = upper_legs_mass / flyer_params["upper_legs_length"] * pow(flyer_params["upper_legs_length"],3)/3.0;
    flyer.upper_legs = Body::Rigid(MassProperties(upper_legs_mass, Vec3(0.0, upper_legs_com, 0.0), Inertia(upper_legs_inertia)));
    flyer.upper_legs.addDecoration(Transform(Vec3(0.0,-flyer_params["upper_legs_length"]/2.0, -flyer_params["torso_width"]/2.0+flyer_params["upper_legs_radius"])), 
                                  DecorativeCylinder(flyer_params["upper_legs_radius"], flyer_params["upper_legs_length"]/2.0));
    flyer.upper_legs.addDecoration(Transform(Vec3(0.0,-flyer_params["upper_legs_length"]/2.0, flyer_params["torso_width"]/2.0-flyer_params["upper_legs_radius"])), 
                                  DecorativeCylinder(flyer_params["upper_legs_radius"], flyer_params["upper_legs_length"]/2.0));

    // mass distributed lower legs
    double lower_legs_mass = 2.0 * flyer_params["body_density"] * flyer_params["lower_legs_length"] * 
        M_PI * flyer_params["lower_legs_radius"] * flyer_params["lower_legs_radius"];
    double lower_legs_com = -flyer_params["lower_legs_length"] / 2.0;
    double lower_legs_inertia = lower_legs_mass / flyer_params["lower_legs_length"] * pow(flyer_params["lower_legs_length"],3)/3.0;
    flyer.lower_legs = Body::Rigid(MassProperties(lower_legs_mass, Vec3(0.0, lower_legs_com, 0.0), Inertia(lower_legs_inertia)));
    flyer.lower_legs.addDecoration(Transform(Vec3(0.0,-flyer_params["lower_legs_length"]/2.0, -flyer_params["torso_width"]/2.0+flyer_params["upper_legs"])), 
                                  DecorativeCylinder(flyer_params["lower_legs_radius"], flyer_params["lower_legs_length"]/2.0));
    flyer.lower_legs.addDecoration(Transform(Vec3(0.0,-flyer_params["lower_legs_length"]/2.0, flyer_params["torso_width"]/2.0-flyer_params["upper_legs_radius"])), 
                                  DecorativeCylinder(flyer_params["lower_legs_radius"], flyer_params["lower_legs_length"]/2.0));

    std::cout << "flyer height " <<flyer_params["lower_legs_length"]+flyer_params["upper_legs_length"]+flyer_params["torso_length"]+
        flyer_params["neck_length"]+flyer_params["head_radius"]*2 << 
        " arms " << flyer_params["upper_arms_length"]+flyer_params["lower_arms_length"] << std::endl;
    std::cout << "total mass " << lower_arms_mass+upper_arms_mass+torso_mass+head_mass+upper_legs_mass+lower_legs_mass << std::endl;

    // Yes, Pin is a mobilized body, but really it behaves like the hinge, so name it after the hinge bodypart
    flyer.hands = MobilizedBody::Pin (lines_anchor, Transform(Vec3(0,-lines_length,0.0)),
        flyer.lower_arms, Transform(Vec3(0, 0, 0)));
    flyer.elbows = MobilizedBody::Pin(flyer.hands, Transform(Vec3(0,-flyer_params["lower_arms_length"],0.0)),
        flyer.upper_arms, Transform(Vec3(0, 0, 0)));
    flyer.shoulders = MobilizedBody::Pin(flyer.elbows, Transform(Vec3(0,-flyer_params["upper_arms_length"],0.0)),
        flyer.torso_head, Transform(Vec3(0, 0, 0)));
    flyer.hips = MobilizedBody::Pin(flyer.shoulders, Transform(Vec3(0,-flyer_params["torso_length"]+flyer_params["upper_arms_radius"],0.0)),
        flyer.upper_legs, Transform(Vec3(0, 0, 0)));
    flyer.knees = MobilizedBody::Pin(flyer.hips, Transform(Vec3(0,-flyer_params["upper_legs_length"],0.0)),
        flyer.lower_legs, Transform(Vec3(0, 0, 0)));

    typedef std::pair<MobilizedBody *,double> JointTorque;
    std::map<std::string,JointTorque> joint_map;
    joint_map["hands"] = JointTorque(&flyer.hands,flyer_params["hands_max_torque"]);
    joint_map["elbows"] = JointTorque(&flyer.elbows,flyer_params["elbows_max_torque"]);
    joint_map["shoulders"] = JointTorque(&flyer.shoulders,flyer_params["shoulders_max_torque"]);
    joint_map["hips"] = JointTorque(&flyer.hips,flyer_params["hips_max_torque"]);
    joint_map["knees"] = JointTorque(&flyer.knees,flyer_params["knees_max_torque"]);
    joints = Joints(joint_map);

    // damping in all joints
    flyer.hands_damper = SimTK::Force::MobilityLinearDamper::MobilityLinearDamper(forces, flyer.hands, MobilizerUIndex(0), flyer_params["joint_damping"]);
    flyer.elbows_damper = SimTK::Force::MobilityLinearDamper::MobilityLinearDamper(forces, flyer.elbows, MobilizerUIndex(0), flyer_params["joint_damping"]);
    flyer.shoulders_damper = SimTK::Force::MobilityLinearDamper::MobilityLinearDamper(forces, flyer.shoulders, MobilizerUIndex(0), flyer_params["joint_damping"]);
    flyer.hips_damper = SimTK::Force::MobilityLinearDamper::MobilityLinearDamper(forces, flyer.hips, MobilizerUIndex(0), flyer_params["joint_damping"]);
    flyer.knees_damper = SimTK::Force::MobilityLinearDamper::MobilityLinearDamper(forces, flyer.knees, MobilizerUIndex(0), flyer_params["joint_damping"]);

    // range of motion of all joints
    double joint_range_damping = flyer_params["joint_range_damping"], joint_range_stiffness = flyer_params["joint_range_stiffness"];
    flyer.hands_range_limit = SimTK::Force::MobilityLinearStop::MobilityLinearStop(forces, flyer.hands, MobilizerQIndex(0), 
        joint_range_stiffness, joint_range_damping, flyer_params["hands_lower_limit"]*M_PI/180.0, flyer_params["hands_upper_limit"]*M_PI/180.0);
    flyer.elbows_range_limit = SimTK::Force::MobilityLinearStop::MobilityLinearStop(forces, flyer.elbows, MobilizerQIndex(0), 
        joint_range_stiffness, joint_range_damping, flyer_params["elbows_lower_limit"]*M_PI/180.0, flyer_params["elbows_upper_limit"]*M_PI/180.0);
    flyer.shoulders_range_limit = SimTK::Force::MobilityLinearStop::MobilityLinearStop(forces, flyer.shoulders, MobilizerQIndex(0), 
        joint_range_stiffness, joint_range_damping, flyer_params["shoulders_lower_limit"]*M_PI/180.0, flyer_params["shoulders_upper_limit"]*M_PI/180.0);
    flyer.hips_range_limit = SimTK::Force::MobilityLinearStop::MobilityLinearStop(forces, flyer.hips, MobilizerQIndex(0), 
        joint_range_stiffness, joint_range_damping, flyer_params["hips_lower_limit"]*M_PI/180.0, flyer_params["hips_upper_limit"]*M_PI/180.0);
    flyer.knees_range_limit = SimTK::Force::MobilityLinearStop::MobilityLinearStop(forces, flyer.knees, MobilizerQIndex(0), 
        joint_range_stiffness, joint_range_damping, flyer_params["knees_lower_limit"]*M_PI/180.0, flyer_params["knees_upper_limit"]*M_PI/180.0);

    flyer.hands_max_torque = flyer_params["hands_max_torque"];
    flyer.elbows_max_torque = flyer_params["elbows_max_torque"];
    flyer.shoulders_max_torque = flyer_params["shoulders_max_torque"];
    flyer.hips_max_torque = flyer_params["hips_max_torque"];
    flyer.knees_max_torque = flyer_params["knees_max_torque"];
}

int main(int argc, char *argv[]) {
    double slow_mo_rate, gravity_accel;
    std::string rig_filename, flyer_filename, initial_state_filename, poses_filename;

    parse_args(argc, argv, &slow_mo_rate, &gravity_accel, &rig_filename, &flyer_filename, &initial_state_filename, &poses_filename);

    // Create the system.
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);

    Force::UniformGravity gravity(forces, matter, Vec3(0, -gravity_accel, 0)); 

    // rig properties
    std::map<std::string, double> rig_params;
    read_map(rig_filename, rig_params);


    Rig rig;
    create_rig(matter.Ground(), rig_params, forces, rig);

    std::map<std::string, double> flyer_params;
    read_map(flyer_filename, flyer_params);

    // create rigid bodies
    Flyer flyer;

    Joints joints;
    create_flyer(rig.lines_mobile, rig_params["lines_length"], flyer_params, forces, joints, flyer);

    double fps = 120.0;
    double dt = 1.0/fps;

    // read poses before almost everything UI related (and before topology is realized)
    Pose pose(poses_filename, flyer_params, joints, forces);

    // Set up visualization.
    Visualizer viz(system);
    // camera
    viz.setCameraTransform(Transform(Rotation(14.0*M_PI/180.0,CoordinateAxis(0)),Vec3(0.0, 3.0, 10.0)));
    viz.setCameraFieldOfView(0.8);
    // real time, slow-mo if needed
    viz.setMode(SimTK::Visualizer::RealTime);
    viz.setRealTimeScale(slow_mo_rate);

    system.addEventReporter(new Visualizer::Reporter(viz, dt));

    // from ChainExample.cpp
    MyListener*            listener = new MyListener(&pose);
    Visualizer::InputSilo* silo = new Visualizer::InputSilo();
    viz.addInputListener(listener); // order matters here
    viz.addInputListener(silo);
    system.addEventHandler
       (new UserInputHandler(viz,*silo, Real(0.01))); // check input every 10ms
    system.addEventHandler
       (new PoseForceUpdater(&pose, Real(0.001))); // update forces every 1 ms

    // Initialize the system and state.
    system.realizeTopology ();
    State state = system.getDefaultState();

    // set initial position
    double initial_lines_pos, initial_lines_vel;
    std::map<std::string, double> initial_joint_pos, initial_joint_vel;
    std::string initial_pose_name;
    read_sys_state(initial_state_filename, joints.index, initial_lines_pos, initial_lines_vel, initial_joint_pos, initial_joint_vel, initial_pose_name);
    set_sys_state(state, initial_lines_pos, initial_lines_vel, initial_joint_pos, initial_joint_vel, &rig.lines_mobile, joints);
    pose.set_pose(initial_pose_name);

    // Simulate it.
    RungeKuttaMersonIntegrator integ(system);
    TimeStepper ts(system, integ);
    ts.initialize (state);
    ts.stepTo(60);
}
