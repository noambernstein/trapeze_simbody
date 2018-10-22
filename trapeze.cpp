#include "Simbody.h"
#include <cmath>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <map>
#include <fstream>
using namespace SimTK;

#define N_JOINTS 6

void read_map(std::string filename, std::map<std::string,double> &param_map) {
    std::ifstream inFile;
    inFile.open(filename);

    std::string key, value;
    while (inFile >> key >> value) {
        param_map[key] = std::stod(value);
    }

    inFile.close();
}

void args_to_map(int argc, char *argv[], std::map<std::string,std::string> &args_map) {
    for (int i=1; i < argc; i++) {
        if (i < argc-1) {
            args_map[std::string(argv[i])] = std::string(argv[i+1]);
            i += 1;
        } else {
            std::cerr << "Got argument number "<<i<<" '"<<argv[i]<<"' but no following value" << std::endl;
            exit(1);
        }
    }
}

void parse_args(int argc, char *argv[], double *slow_mo_rate, 
    std::string *rig_filename, std::string *flyer_filename, std::string *initial_state_filename) {

    std::map<std::string,std::string> args_map;

    args_to_map(argc, argv, args_map);

    if (args_map.find("--slow") == args_map.end()) {
        *slow_mo_rate = 1.0;
    } else {
        *slow_mo_rate = 1.0/std::stod(args_map["--slow"]);
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
        *initial_state_filename = "initial_state.data";
    } else {
        *initial_state_filename = args_map["--initial_state"];
    }

}

void read_joint_state(std::string filename, std::map<std::string,int> joint_index, double *joint_pos, double *joint_vel) {
    std::ifstream inFile;
    std::string key, pos_value, vel_value;

    std::cout << "reading initial state from "<<filename<<std::endl;

    for (int i=0; i < joint_index.size(); i++) {
        joint_pos[i] = 0.0;
        joint_vel[i] = 0.0;
    }

    inFile.open(filename);
    while (inFile >> key >> pos_value >> vel_value) {
        joint_pos[joint_index[key]] = std::stod(pos_value)*M_PI/180.0;
        joint_vel[joint_index[key]] = std::stod(vel_value)*M_PI/180.0;
    }
    inFile.close();

}

void set_joint_state(State *state, double *pos, double *vel, std::vector<MobilizedBody::Pin *> joints) {
    for (int i=0; i < joints.size(); i++) {
        joints[i]->setAngle(*state, pos[i]);
        joints[i]->setRate(*state, vel[i]);
    }
}

// This is a custom InputListener. We'll register it prior to the InputSilo so
// that we can intercept all input and say something about it. No input processing
// is done here other than that, and we pass on everything we receive down the
// chain to the next listener (which will be an InputSilo in this case).
class MyListener : public Visualizer::InputListener {
public:
    MyListener() {}

    ~MyListener() {}

    virtual bool keyPressed(unsigned key, unsigned modifier) override {
        String mod;
        if (modifier&ControlIsDown) mod += "CTRL ";
        if (modifier&ShiftIsDown) mod += "SHIFT ";
        if (modifier&AltIsDown)  mod += "ALT ";

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

        return false; // key passed on
    }

    virtual bool sliderMoved(int whichSlider, Real value) override {
        printf("Listener sees slider %d now at %g\n", whichSlider, value);
        return false;   // slider move passed on
    }

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

int main(int argc, char *argv[]) {
    double slow_mo_rate;
    std::string rig_filename, flyer_filename, initial_state_filename;

    parse_args(argc, argv, &slow_mo_rate, &rig_filename, &flyer_filename, &initial_state_filename);

    // Create the system.
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);

    Force::UniformGravity gravity(forces, matter, Vec3(0, -9.8, 0)); 

    // rig properties
    std::map<std::string, double> rig_params;
    read_map(rig_filename, rig_params);

    Body::Rigid fly_crane_body(MassProperties(1.0, Vec3(0.0), Inertia(1.0)));
    fly_crane_body.addDecoration(Transform(), DecorativeBrick(Vec3(rig_params["fly_crane_thickness"]/2.0,
        rig_params["fly_crane_thickness"]/2.0,rig_params["fly_crane_width"])));
    MobilizedBody::Pin fly_crane(matter.Ground(), Transform(Vec3(0.0, rig_params["fly_crane_height"]+rig_params["fly_crane_thickness"]/2.0, 0.0)),
        fly_crane_body, Transform(Vec3(0, 0, 0)));

    PolygonalMesh net_mesh;
    net_mesh.loadFile("net.obj");
    Body::Rigid net_body(MassProperties(1.0, Vec3(0.0), Inertia(1.0)));
    DecorativeMesh net_mesh_decor(net_mesh);
    net_mesh_decor.setRepresentation(DecorativeGeometry::DrawWireframe);
    net_mesh_decor.setLineThickness(2.0);
    net_mesh_decor.setColor(Vec3(1.0, 1.0, 1.0));
    net_body.addDecoration(Transform(), net_mesh_decor);
    MobilizedBody::Pin net(matter.Ground(), Transform(Vec3(rig_params["net_offset"], rig_params["net_height"], 0.0)),
        net_body, Transform(Vec3(0, 0, 0)));

    // mass distributed lines + point mass bar
    double lines_bar_com = -(rig_params["lines_mass"] * rig_params["lines_length"]/2.0 + 
                             rig_params["bar_mass"]*rig_params["lines_length"])/(rig_params["lines_mass"]+rig_params["bar_mass"]);
    double lines_bar_inertia = (rig_params["lines_mass"]/rig_params["lines_length"]) * pow(rig_params["lines_length"],3)/3.0 + 
                                rig_params["bar_mass"] * pow(rig_params["lines_length"],3)/3.0;
    Body::Rigid lines_bar_body(MassProperties(rig_params["lines_mass"]+rig_params["bar_mass"], Vec3(0.0, lines_bar_com, 0.0), 
        Inertia(lines_bar_inertia)));
    lines_bar_body.addDecoration(Transform(), DecorativeLine(Vec3(0.0, 0.0, -rig_params["bar_length"]/2.0), 
        Vec3(0.0, -rig_params["lines_length"], -rig_params["bar_length"]/2.0)));
    lines_bar_body.addDecoration(Transform(), DecorativeLine(Vec3(0.0, 0.0,  rig_params["bar_length"]/2.0), 
        Vec3(0.0, -rig_params["lines_length"],  rig_params["bar_length"]/2.0)));
    lines_bar_body.addDecoration(Transform(Rotation(90.0*M_PI/180.0,CoordinateAxis(2)),Vec3(0.0, -rig_params["lines_length"], 0.0)), 
                                 DecorativeCylinder(rig_params["bar_radius"], rig_params["bar_length"]/2.0));

    std::map<std::string, double> flyer_params;
    read_map(flyer_filename, flyer_params);

    // mass distributed lower arms
    double lower_arms_mass = 2.0 * flyer_params["body_density"] * flyer_params["lower_arms_length"] * M_PI * flyer_params["lower_arms_radius"] * flyer_params["lower_arms_radius"];
    double lower_arms_com = -flyer_params["lower_arms_length"] / 2.0;
    double lower_arms_inertia = lower_arms_mass / flyer_params["lower_arms_length"] * pow(flyer_params["lower_arms_length"],3)/3.0;
    Body::Rigid lower_arms_body(MassProperties(lower_arms_mass, Vec3(0.0, lower_arms_com, 0.0), Inertia(lower_arms_inertia)));
    lower_arms_body.addDecoration(Transform(Vec3(0.0,-flyer_params["lower_arms_length"]/2.0, -flyer_params["torso_width"]/2.0-flyer_params["upper_arms_radius"])), 
                                  DecorativeCylinder(flyer_params["lower_arms_radius"], flyer_params["lower_arms_length"]/2.0));
    lower_arms_body.addDecoration(Transform(Vec3(0.0,-flyer_params["lower_arms_length"]/2.0, flyer_params["torso_width"]/2.0+flyer_params["upper_arms_radius"])), 
                                  DecorativeCylinder(flyer_params["lower_arms_radius"], flyer_params["lower_arms_length"]/2.0));

    // mass distributed upper arms
    double upper_arms_mass = 2.0 * flyer_params["body_density"] * flyer_params["upper_arms_length"] * M_PI * flyer_params["upper_arms_radius"] * flyer_params["upper_arms_radius"];
    double upper_arms_com = -flyer_params["upper_arms_length"] / 2.0;
    double upper_arms_inertia = upper_arms_mass / flyer_params["upper_arms_length"] * pow(flyer_params["upper_arms_length"],3)/3.0;
    Body::Rigid upper_arms_body(MassProperties(upper_arms_mass, Vec3(0.0, upper_arms_com, 0.0), Inertia(upper_arms_inertia)));
    upper_arms_body.addDecoration(Transform(Vec3(0.0,-flyer_params["upper_arms_length"]/2.0, -flyer_params["torso_width"]/2.0-flyer_params["upper_arms_radius"])), 
                                  DecorativeCylinder(flyer_params["upper_arms_radius"], flyer_params["upper_arms_length"]/2.0));
    upper_arms_body.addDecoration(Transform(Vec3(0.0,-flyer_params["upper_arms_length"]/2.0, flyer_params["torso_width"]/2.0+flyer_params["upper_arms_radius"])), 
                                  DecorativeCylinder(flyer_params["upper_arms_radius"], flyer_params["upper_arms_length"]/2.0));

    // mass distributed torso+head
    double torso_mass = flyer_params["body_density"] * flyer_params["torso_length"] * flyer_params["torso_width"] * flyer_params["torso_depth"];
    double head_mass = flyer_params["body_density"] * (4.0/3.0) * M_PI * pow(flyer_params["head_radius"],3);
    // relative to point flyer_params["upper_arms_radius"] below top of torso
    double torso_head_com = (-torso_mass * (flyer_params["torso_length"]/2.0-flyer_params["upper_arms_radius"]) + head_mass * (flyer_params["upper_arms_radius"]+flyer_params["neck_length"]+flyer_params["head_radius"])) / 
                            (torso_mass + head_mass);
    double torso_inertia_com = (1.0/12.0) * torso_mass * (flyer_params["torso_length"]*flyer_params["torso_length"] + flyer_params["torso_depth"]*flyer_params["torso_depth"]);
    double head_inertia_com = (2.0/5.0) * head_mass * flyer_params["head_radius"]*flyer_params["head_radius"];
    double torso_head_inertia = torso_inertia_com + torso_mass*pow(flyer_params["torso_length"]/2.0-flyer_params["upper_arms_radius"],3) +
                                head_inertia_com + head_mass*pow(flyer_params["upper_arms_radius"]+flyer_params["neck_length"]+flyer_params["head_radius"],2);
    Body::Rigid torso_head_body(MassProperties(torso_mass+head_mass, Vec3(0.0, torso_head_com, 0.0), Inertia(torso_head_inertia)));
    torso_head_body.addDecoration(Transform(Vec3(0.0,-flyer_params["torso_length"]/2.0+flyer_params["upper_arms_radius"], 0.0)), 
        DecorativeBrick(Vec3(flyer_params["torso_depth"]/2.0, flyer_params["torso_length"]/2.0, flyer_params["torso_width"]/2.0)));
    torso_head_body.addDecoration(Transform(Vec3(0.0,flyer_params["neck_length"]+flyer_params["head_radius"]+flyer_params["upper_arms_radius"], 0.0)), DecorativeSphere(flyer_params["head_radius"]));

    // mass distributed upper legs
    double upper_legs_mass = 2.0 * flyer_params["body_density"] * flyer_params["upper_legs_length"] * M_PI * flyer_params["upper_legs_radius"] * flyer_params["upper_legs_radius"];
    double upper_legs_com = -flyer_params["upper_legs_length"] / 2.0;
    double upper_legs_inertia = upper_legs_mass / flyer_params["upper_legs_length"] * pow(flyer_params["upper_legs_length"],3)/3.0;
    Body::Rigid upper_legs_body(MassProperties(upper_legs_mass, Vec3(0.0, upper_legs_com, 0.0), Inertia(upper_legs_inertia)));
    upper_legs_body.addDecoration(Transform(Vec3(0.0,-flyer_params["upper_legs_length"]/2.0, -flyer_params["torso_width"]/2.0+flyer_params["upper_legs_radius"])), 
                                  DecorativeCylinder(flyer_params["upper_legs_radius"], flyer_params["upper_legs_length"]/2.0));
    upper_legs_body.addDecoration(Transform(Vec3(0.0,-flyer_params["upper_legs_length"]/2.0, flyer_params["torso_width"]/2.0-flyer_params["upper_legs_radius"])), 
                                  DecorativeCylinder(flyer_params["upper_legs_radius"], flyer_params["upper_legs_length"]/2.0));

    // mass distributed lower legs
    double lower_legs_mass = 2.0 * flyer_params["body_density"] * flyer_params["lower_legs_length"] * M_PI * flyer_params["lower_legs_radius"] * flyer_params["lower_legs_radius"];
    double lower_legs_com = -flyer_params["lower_legs_length"] / 2.0;
    double lower_legs_inertia = lower_legs_mass / flyer_params["lower_legs_length"] * pow(flyer_params["lower_legs_length"],3)/3.0;
    Body::Rigid lower_legs_body(MassProperties(lower_legs_mass, Vec3(0.0, lower_legs_com, 0.0), Inertia(lower_legs_inertia)));
    lower_legs_body.addDecoration(Transform(Vec3(0.0,-flyer_params["lower_legs_length"]/2.0, -flyer_params["torso_width"]/2.0+flyer_params["upper_legs_radius"])), 
                                  DecorativeCylinder(flyer_params["lower_legs_radius"], flyer_params["lower_legs_length"]/2.0));
    lower_legs_body.addDecoration(Transform(Vec3(0.0,-flyer_params["lower_legs_length"]/2.0, flyer_params["torso_width"]/2.0-flyer_params["upper_legs_radius"])), 
                                  DecorativeCylinder(flyer_params["lower_legs_radius"], flyer_params["lower_legs_length"]/2.0));

    std::cout << "flyer height " <<flyer_params["lower_legs_length"]+flyer_params["upper_legs_length"]+flyer_params["torso_length"]+flyer_params["neck_length"]+flyer_params["head_radius"]*2 << 
        " arms " << flyer_params["upper_arms_length"]+flyer_params["lower_arms_length"] << std::endl;
    std::cout << "total mass " << lower_arms_mass+upper_arms_mass+torso_mass+head_mass+upper_legs_mass+lower_legs_mass << std::endl;

    // Yes, Pin is a mobilized body, but really it behaves like the hinge, so name it after the hinge bodypart
    MobilizedBody::Pin lines_anchor(matter.Ground(), Transform(Vec3(0.0, rig_params["fly_crane_height"], 0.0)),
        lines_bar_body, Transform(Vec3(0, 0, 0)));
    MobilizedBody::Pin hands(lines_anchor, Transform(Vec3(0,-rig_params["lines_length"],0.0)),
        lower_arms_body, Transform(Vec3(0, 0, 0)));
    MobilizedBody::Pin elbows(hands, Transform(Vec3(0,-flyer_params["lower_arms_length"],0.0)),
        upper_arms_body, Transform(Vec3(0, 0, 0)));
    MobilizedBody::Pin shoulders(elbows, Transform(Vec3(0,-flyer_params["upper_arms_length"],0.0)),
        torso_head_body, Transform(Vec3(0, 0, 0)));
    MobilizedBody::Pin hips(shoulders, Transform(Vec3(0,-flyer_params["torso_length"]+flyer_params["upper_arms_radius"],0.0)),
        upper_legs_body, Transform(Vec3(0, 0, 0)));
    MobilizedBody::Pin knees(hips, Transform(Vec3(0,-flyer_params["upper_legs_length"],0.0)),
        lower_legs_body, Transform(Vec3(0, 0, 0)));


    std::vector<MobilizedBody::Pin *> joints;
    joints.push_back(&lines_anchor);
    joints.push_back(&hands);
    joints.push_back(&elbows);
    joints.push_back(&shoulders);
    joints.push_back(&hips);
    joints.push_back(&knees);
    std::map<std::string,int> joint_index;
    joint_index[std::string("lines_anchor")] = 0;
    joint_index[std::string("hands")] = 1;
    joint_index[std::string("elbows")] = 2;
    joint_index[std::string("shoulders")] = 3;
    joint_index[std::string("hips")] = 4;
    joint_index[std::string("knees")] = 5;

    double joint_damping=10.0;
    SimTK::Force::MobilityLinearDamper::MobilityLinearDamper lines_fly_crane_damper(forces, lines_anchor, MobilizerUIndex(0), 100.0);
    SimTK::Force::MobilityLinearDamper::MobilityLinearDamper hands_damper(forces, hands, MobilizerUIndex(0), joint_damping);
    SimTK::Force::MobilityLinearDamper::MobilityLinearDamper elbows_damper(forces, elbows, MobilizerUIndex(0), joint_damping);
    SimTK::Force::MobilityLinearDamper::MobilityLinearDamper shoulder_damper(forces, shoulders, MobilizerUIndex(0), joint_damping);
    SimTK::Force::MobilityLinearDamper::MobilityLinearDamper hips_damper(forces, hips, MobilizerUIndex(0), joint_damping);
    SimTK::Force::MobilityLinearDamper::MobilityLinearDamper knees_damper(forces, knees, MobilizerUIndex(0), joint_damping);

    double mobile_anchor=1.0, mobile_hands=1.0, mobile_elbows=1.0, mobile_shoulders=1.0, mobile_hips=1.0, mobile_knees=1.0;
    double joint_range_stiffness=100.0, joint_range_damping=100.0;
    SimTK::Force::MobilityLinearStop::MobilityLinearStop anchor_stop(forces, lines_anchor, MobilizerQIndex(0), 
        joint_range_stiffness, joint_range_damping, -mobile_anchor*180.0*M_PI/180.0, mobile_anchor*180.0*M_PI/180.0);
    SimTK::Force::MobilityLinearStop::MobilityLinearStop hands_stop(forces, hands, MobilizerQIndex(0), 
        joint_range_stiffness, joint_range_damping, -mobile_hands*180.0*M_PI/180.0, mobile_hands*180.0*M_PI/180.0);
    SimTK::Force::MobilityLinearStop::MobilityLinearStop elbows_stop(forces, elbows, MobilizerQIndex(0), 
        joint_range_stiffness, joint_range_damping, mobile_elbows*0.0*M_PI/180.0, mobile_elbows*160.0*M_PI/180.0);
    SimTK::Force::MobilityLinearStop::MobilityLinearStop shoulders_stop(forces, shoulders, MobilizerQIndex(0), 
        joint_range_stiffness, joint_range_damping, -mobile_shoulders*160.0*M_PI/180.0, mobile_shoulders*5.0*M_PI/180.0);
    SimTK::Force::MobilityLinearStop::MobilityLinearStop hips_stop(forces, hips, MobilizerQIndex(0), 
        joint_range_stiffness, joint_range_damping, -mobile_hips*150.0*M_PI/180.0, mobile_hips*10.0*M_PI/180.0);
    SimTK::Force::MobilityLinearStop::MobilityLinearStop knees_stop(forces, knees, MobilizerQIndex(0), 
        joint_range_stiffness, joint_range_damping, mobile_knees*0.0*M_PI/180.0, mobile_knees*150.0*M_PI/180.0);

    double fps = 120.0;
    double dt = 1.0/fps;

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
    MyListener*            listener = new MyListener();
    Visualizer::InputSilo* silo = new Visualizer::InputSilo();
    viz.addInputListener(listener); // order matters here
    viz.addInputListener(silo);
    system.addEventHandler
       (new UserInputHandler(viz,*silo, Real(0.01))); // check input every 10ms

    // Initialize the system and state.
    system.realizeTopology ();
    State state = system.getDefaultState();

    // set initial position
    double initial_joint_pos[N_JOINTS], initial_joint_vel[N_JOINTS];

    read_joint_state(initial_state_filename, joint_index, initial_joint_pos, initial_joint_vel);
    set_joint_state(&state, initial_joint_pos, initial_joint_vel, joints);

    // Simulate it.
    RungeKuttaMersonIntegrator integ(system);
    TimeStepper ts(system, integ);
    ts.initialize (state);
    ts.stepTo(60);
}
