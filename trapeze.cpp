#include "Simbody.h"
#include <cmath>
#include <chrono>
#include <stdio.h>
#include <unistd.h>
using namespace SimTK;
using namespace std::chrono;

int main(int argc, char *argv[]) {
    // Create the system.
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);

    Force::UniformGravity gravity(forces, matter, Vec3(0, -9.8, 0)); 

    // rig properties
    double fly_crane_height = 9.7, fly_crane_width=2.0, fly_crane_thickness=0.1;
    double net_height = 2.5, net_offset=-4.0;
    double lines_length = 3.64, lines_radius = 0.0025, lines_mass = 2.0;
    double bar_length = 0.9, bar_radius=0.01, bar_mass = 3.0;
    double board_height = 6.97, board_offset = 4.45;

    Body::Rigid fly_crane_body(MassProperties(1.0, Vec3(0.0), Inertia(1.0)));
    fly_crane_body.addDecoration(Transform(), DecorativeBrick(Vec3(fly_crane_thickness/2.0,fly_crane_thickness/2.0,fly_crane_width)));
    MobilizedBody::Pin fly_crane(matter.Ground(), Transform(Vec3(0.0, fly_crane_height+fly_crane_thickness/2.0, 0.0)),
        fly_crane_body, Transform(Vec3(0, 0, 0)));

    PolygonalMesh net_mesh;
    net_mesh.loadFile("net.obj");
    Body::Rigid net_body(MassProperties(1.0, Vec3(0.0), Inertia(1.0)));
    net_body.addDecoration(Transform(), DecorativeMesh(net_mesh));
    MobilizedBody::Pin net(matter.Ground(), Transform(Vec3(net_offset, net_height, 0.0)),
        net_body, Transform(Vec3(0, 0, 0)));

    // mass distributed lines + point mass bar
    double lines_bar_com = -(lines_mass * lines_length/2.0 + bar_mass*lines_length)/(lines_mass+bar_mass);
    double lines_bar_inertia = (lines_mass/lines_length) * pow(lines_length,3)/3.0 + bar_mass * pow(lines_length,3)/3.0;
    // printf("%f %f\n", lines_bar_com, lines_bar_inertia);
    Body::Rigid lines_bar_body(MassProperties(lines_mass+bar_mass, Vec3(0.0, lines_bar_com, 0.0), Inertia(lines_bar_inertia)));
    lines_bar_body.addDecoration(Transform(), DecorativeLine(Vec3(0.0, 0.0, -bar_length/2.0), Vec3(0.0, -lines_length, -bar_length/2.0)));
    lines_bar_body.addDecoration(Transform(), DecorativeLine(Vec3(0.0, 0.0,  bar_length/2.0), Vec3(0.0, -lines_length,  bar_length/2.0)));
    lines_bar_body.addDecoration(Transform(Rotation(90.0*M_PI/180.0,CoordinateAxis(2)),Vec3(0.0, -lines_length, 0.0)), 
                                 DecorativeCylinder(bar_radius, bar_length/2.0));

    double body_density = 1000.0;

    double lower_arms_length = 0.3, lower_arms_radius = 0.04;
    double upper_arms_length = 0.3, upper_arms_radius = 0.05;
    double torso_length = 0.576, torso_width = 0.384, torso_depth = 0.1536;
    double head_radius = 0.12;
    double neck_length = 0.03;
    double upper_legs_length = 0.52, upper_legs_radius = 0.08;
    double lower_legs_length = 0.48, lower_legs_radius = 0.06;

    // mass distributed lower arms
    double lower_arms_mass = 2.0 * body_density * lower_arms_length * M_PI * lower_arms_radius * lower_arms_radius;
    double lower_arms_com = -lower_arms_length / 2.0;
    double lower_arms_inertia = lower_arms_mass / lower_arms_length * pow(lower_arms_length,3)/3.0;
    Body::Rigid lower_arms_body(MassProperties(lower_arms_mass, Vec3(0.0, lower_arms_com, 0.0), Inertia(lower_arms_inertia)));
    lower_arms_body.addDecoration(Transform(Vec3(0.0,-lower_arms_length/2.0, -torso_width/2.0-upper_arms_radius)), 
                                  DecorativeCylinder(lower_arms_radius, lower_arms_length/2.0));
    lower_arms_body.addDecoration(Transform(Vec3(0.0,-lower_arms_length/2.0, torso_width/2.0+upper_arms_radius)), 
                                  DecorativeCylinder(lower_arms_radius, lower_arms_length/2.0));

    // mass distributed upper arms
    double upper_arms_mass = 2.0 * body_density * upper_arms_length * M_PI * upper_arms_radius * upper_arms_radius;
    double upper_arms_com = -upper_arms_length / 2.0;
    double upper_arms_inertia = upper_arms_mass / upper_arms_length * pow(upper_arms_length,3)/3.0;
    Body::Rigid upper_arms_body(MassProperties(upper_arms_mass, Vec3(0.0, upper_arms_com, 0.0), Inertia(upper_arms_inertia)));
    upper_arms_body.addDecoration(Transform(Vec3(0.0,-upper_arms_length/2.0, -torso_width/2.0-upper_arms_radius)), 
                                  DecorativeCylinder(upper_arms_radius, upper_arms_length/2.0));
    upper_arms_body.addDecoration(Transform(Vec3(0.0,-upper_arms_length/2.0, torso_width/2.0+upper_arms_radius)), 
                                  DecorativeCylinder(upper_arms_radius, upper_arms_length/2.0));

    // mass distributed torso+head
    double torso_mass = body_density * torso_length * torso_width * torso_depth;
    double head_mass = body_density * (4.0/3.0) * M_PI * pow(head_radius,3);
    // relative to point upper_arms_radius below top of torso
    double torso_head_com = (-torso_mass * (torso_length/2.0-upper_arms_radius) + head_mass * (upper_arms_radius+neck_length+head_radius)) / 
                            (torso_mass + head_mass);
    double torso_inertia_com = (1.0/12.0) * torso_mass * (torso_length*torso_length + torso_depth*torso_depth);
    double head_inertia_com = (2.0/5.0) * head_mass * head_radius*head_radius;
    double torso_head_inertia = torso_inertia_com + torso_mass*pow(torso_length/2.0-upper_arms_radius,3) +
                                head_inertia_com + head_mass*pow(upper_arms_radius+neck_length+head_radius,2);
    Body::Rigid torso_head_body(MassProperties(torso_mass+head_mass, Vec3(0.0, torso_head_com, 0.0), Inertia(torso_head_inertia)));
    torso_head_body.addDecoration(Transform(Vec3(0.0,-torso_length/2.0+upper_arms_radius, 0.0)), 
        DecorativeBrick(Vec3(torso_depth/2.0, torso_length/2.0, torso_width/2.0)));
    torso_head_body.addDecoration(Transform(Vec3(0.0,neck_length+head_radius+upper_arms_radius, 0.0)), DecorativeSphere(head_radius));

    // mass distributed upper legs
    double upper_legs_mass = 2.0 * body_density * upper_legs_length * M_PI * upper_legs_radius * upper_legs_radius;
    double upper_legs_com = -upper_legs_length / 2.0;
    double upper_legs_inertia = upper_legs_mass / upper_legs_length * pow(upper_legs_length,3)/3.0;
    Body::Rigid upper_legs_body(MassProperties(upper_legs_mass, Vec3(0.0, upper_legs_com, 0.0), Inertia(upper_legs_inertia)));
    upper_legs_body.addDecoration(Transform(Vec3(0.0,-upper_legs_length/2.0, -torso_width/2.0+upper_legs_radius)), 
                                  DecorativeCylinder(upper_legs_radius, upper_legs_length/2.0));
    upper_legs_body.addDecoration(Transform(Vec3(0.0,-upper_legs_length/2.0, torso_width/2.0-upper_legs_radius)), 
                                  DecorativeCylinder(upper_legs_radius, upper_legs_length/2.0));

    // mass distributed lower legs
    double lower_legs_mass = 2.0 * body_density * lower_legs_length * M_PI * lower_legs_radius * lower_legs_radius;
    double lower_legs_com = -lower_legs_length / 2.0;
    double lower_legs_inertia = lower_legs_mass / lower_legs_length * pow(lower_legs_length,3)/3.0;
    Body::Rigid lower_legs_body(MassProperties(lower_legs_mass, Vec3(0.0, lower_legs_com, 0.0), Inertia(lower_legs_inertia)));
    lower_legs_body.addDecoration(Transform(Vec3(0.0,-lower_legs_length/2.0, -torso_width/2.0+upper_legs_radius)), 
                                  DecorativeCylinder(lower_legs_radius, lower_legs_length/2.0));
    lower_legs_body.addDecoration(Transform(Vec3(0.0,-lower_legs_length/2.0, torso_width/2.0-upper_legs_radius)), 
                                  DecorativeCylinder(lower_legs_radius, lower_legs_length/2.0));

    printf("flyer height %f arms %f\n", lower_legs_length+upper_legs_length+torso_length+neck_length+head_radius*2, 
        upper_arms_length+lower_arms_length);
    printf("total mass %f\n", lower_arms_mass+upper_arms_mass+torso_mass+head_mass+upper_legs_mass+lower_legs_mass);

    MobilizedBody::Pin lines_bar(matter.Ground(), Transform(Vec3(0.0, fly_crane_height, 0.0)),
        lines_bar_body, Transform(Vec3(0, 0, 0)));
    MobilizedBody::Pin lower_arms(lines_bar, Transform(Vec3(0,-lines_length,0.0)),
        lower_arms_body, Transform(Vec3(0, 0, 0)));
    MobilizedBody::Pin upper_arms(lower_arms, Transform(Vec3(0,-lower_arms_length,0.0)),
        upper_arms_body, Transform(Vec3(0, 0, 0)));
    MobilizedBody::Pin torso_head(upper_arms, Transform(Vec3(0,-upper_arms_length,0.0)),
        torso_head_body, Transform(Vec3(0, 0, 0)));
    MobilizedBody::Pin upper_legs(torso_head, Transform(Vec3(0,-torso_length+upper_arms_radius,0.0)),
        upper_legs_body, Transform(Vec3(0, 0, 0)));
    MobilizedBody::Pin lower_legs(upper_legs, Transform(Vec3(0,-upper_legs_length,0.0)),
        lower_legs_body, Transform(Vec3(0, 0, 0)));

    double damping=200.0;
    SimTK::Force::MobilityLinearDamper::MobilityLinearDamper lines_fly_crane_damper(forces, lines_bar, MobilizerUIndex(0), 20.0);
    SimTK::Force::MobilityLinearDamper::MobilityLinearDamper hands_damper(forces, lower_arms, MobilizerUIndex(0), damping);
    SimTK::Force::MobilityLinearDamper::MobilityLinearDamper elbows_damper(forces, upper_arms, MobilizerUIndex(0), damping);
    SimTK::Force::MobilityLinearDamper::MobilityLinearDamper shoulder_damper(forces, torso_head, MobilizerUIndex(0), damping);
    SimTK::Force::MobilityLinearDamper::MobilityLinearDamper hips_damper(forces, upper_legs, MobilizerUIndex(0), damping);
    SimTK::Force::MobilityLinearDamper::MobilityLinearDamper knees_damper(forces, lower_legs, MobilizerUIndex(0), damping);

    double fps = 60.0;
    double dt = 1.0/fps;

    // Set up visualization.
    Visualizer viz(system);
    viz.setMode(SimTK::Visualizer::RealTime);
    system.addEventReporter(new Visualizer::Reporter(viz, dt));

    // Initialize the system and state.
    system.realizeTopology ();
    State state = system.getDefaultState();
    lines_bar.setRate(state, 1.0);
    lower_arms.setRate(state, 5.0);
    upper_arms.setRate(state, -5.0);

    // Simulate it.
    RungeKuttaMersonIntegrator integ(system);
    TimeStepper ts(system, integ);
    ts.initialize (state);
    ts.stepTo(60);
}
