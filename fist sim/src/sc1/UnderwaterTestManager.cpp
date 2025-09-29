#include "UnderwaterTestManager.h"

#include "UnderwaterTestApp.h"
#include <core/FeatherstoneRobot.h>
#include <entities/statics/Plane.h>
#include <entities/statics/Obstacle.h>
#include <entities/solids/Polyhedron.h>
#include <entities/solids/Box.h>
#include <entities/solids/Sphere.h>
#include <entities/solids/Torus.h>
#include <entities/solids/Cylinder.h>
#include <entities/solids/Compound.h>
#include <entities/solids/Wing.h>
#include <graphics/OpenGLPointLight.h>
#include <graphics/OpenGLSpotLight.h>
#include <graphics/OpenGLTrackball.h>
#include <utils/SystemUtil.hpp>
#include <entities/statics/Obstacle.h>
#include <entities/statics/Terrain.h>
#include <actuators/Thruster.h>
#include <actuators/Servo.h>
#include <actuators/VariableBuoyancy.h>
#include <sensors/scalar/Pressure.h>
#include <sensors/scalar/Odometry.h>
#include <sensors/scalar/DVL.h>
#include <sensors/scalar/Compass.h>
#include <sensors/scalar/IMU.h>
#include <sensors/scalar/GPS.h>
#include <sensors/Contact.h>
#include <sensors/vision/ColorCamera.h>
#include <sensors/vision/DepthCamera.h>
#include <sensors/vision/Multibeam2.h>
#include <sensors/vision/FLS.h>
#include <sensors/vision/SSS.h>
#include <sensors/vision/MSIS.h>
#include <comms/AcousticModem.h>
#include <sensors/Sample.h>
#include <actuators/Light.h>
#include <sensors/scalar/RotaryEncoder.h>
#include <sensors/scalar/Accelerometer.h>
#include <entities/FeatherstoneEntity.h>
#include <entities/forcefields/Trigger.h>
#include <entities/forcefields/Pipe.h>
#include <entities/forcefields/Jet.h>
#include <entities/forcefields/Uniform.h>
#include <entities/AnimatedEntity.h>
#include <sensors/scalar/Profiler.h>
#include <sensors/scalar/Multibeam.h>
#include <utils/UnitSystem.h>
#include <core/ScenarioParser.h>
#include <core/NED.h>
#include <Stonefish/entities/solids/Polyhedron.h>
#include <core/GeneralRobot.h>
#include <LinearMath/btTransform.h>
#include <Stonefish/actuators/Push.h>

UnderwaterTestManager::UnderwaterTestManager(sf::Scalar stepsPerSecond)
: SimulationManager(stepsPerSecond, sf::SolverType::SOLVER_SI, sf::CollisionFilteringType::COLLISION_EXCLUSIVE)
{
}

void UnderwaterTestManager::BuildScenario()
{
    // -------------- MATERIALS--------------
    CreateMaterial("Neutral", sf::UnitSystem::Density(sf::CGS, sf::MKS, 0.9), 0.3);
    SetMaterialsInteraction("ROV_Material", "ROV_Material", 0.5, 0.2);
    CreateMaterial("Rock", sf::UnitSystem::Density(sf::CGS, sf::MKS, 3.0), 0.6);
    
    // -------------- LOOKS --------------
    CreateLook("white", sf::Color::Gray(1.f), 0.9f, 0.0f, 0.f);
    CreateLook("seabed", sf::Color::RGB(0.7f, 0.7f, 0.5f), 0.9f, 0.f, 0.f, "", sf::GetDataPath() + "sand_normal.png");


    // -------------- OCEAN --------------
    EnableOcean(0.0);
    getOcean()->EnableCurrents();
    getAtmosphere()->SetSunPosition(0.0, 45.0);

    // -------------- Physics --------------
    sf::BodyPhysicsSettings phy;
    phy.mode = sf::BodyPhysicsMode::SUBMERGED;
    phy.collisions = true;
    phy.buoyancy = true;
    

    // -------------- Positions --------------
    
    // From Blender Coordinates to NED: [x, y, z] -> [x, z, -y]
    auto to_cg      = sf::Vector3(0, 0, 0);
    auto to_mbs     = sf::Vector3(-0.147759, 0.0, 0.225529);
    auto to_ping    = sf::Vector3(0.024241, 0.12, 0.255529);
    
    auto rel_mbs = to_cg - to_mbs;
    auto rel_ping = to_cg - to_ping;
    
    
    // -------------- DEFINING THE VEHICLE --------------

    phy.buoyancy = false; // External part. Kun visuelt og fysikk for drag.
    sf::Polyhedron* vehicle = new sf::Polyhedron(
        "ROV_",                                              // navn
        phy,                                                // BodyPhysicsSettings
        sf::GetDataPath() + "BlueROV/ResiFarmBlueRov.obj",  // Vsible object
        1.0,                                                // scale
        sf::Transform(sf::IQ(), to_cg),                     // origin transform
        sf::GetDataPath() + "sphere_R=1.obj",               // Physical object. Determines drag etc.
        1.0,
        sf::I4(),
        "Neutral",              // material
        "white",                 // look (can be set to a texture)
        0.005 // Thickness
    );

    // Box defining interia etc.
    phy.buoyancy = true;
    sf::Box* box = new sf::Box("box", phy, sf::Vector3(0.5, 0.5, 0.5), sf::I4(), "Neutral", "white");

    sf::Compound* comp = new sf::Compound("ROV", phy, vehicle, sf::I4());
    comp->AddInternalPart(box, sf::Transform(sf::IQ(), sf::Vector3(0.0, 0.0, 0.0)));
    comp->setDisplayInternalParts(false);

    sf::Robot* robot = new sf::GeneralRobot("Robot", false);
    std::vector<sf::SolidEntity*> links; 
    robot->DefineLinks(comp, links);
    robot->BuildKinematicStructure();

    // -------------- SENSORS --------------

    sf::FLS* fls = new sf::FLS("FLS", 512, 500, 120.0, 30.0, 0.5, 10.0, sf::ColorMap::HOT);
    fls->setGain(1.1);
    fls->setNoise(0.01, 0.02);
    fls->setDisplayOnScreen(true, 900, 250, 0.4f);
    robot->AddVisionSensor(fls, "ROV", sf::Transform(sf::Quaternion(M_PI_2, 0, M_PI_2), rel_mbs));

    sf::MSIS* msis = new sf::MSIS(
        "MSIS", // Name
        0.25,   // Step angle
        500,    // num bins
        2.0,    // horizontal beam width
        30.0,   // verical beam width
        -180.0,  // min rotation
        180.0,   // max rotation
        0.5,    // min range
        10.0,   // max range
        sf::ColorMap::HOT
    );
    msis->setGain(1.5);
    msis->setNoise(0.02, 0.03);
    robot->AddVisionSensor(msis, "ROV", sf::Transform(sf::Quaternion(0, 0, M_PI_2), rel_ping));


    // -------------- ACTUATORS --------------
    sf::Push* pushForward = new sf::Push("PushForward", false);
    pushForward->setForceLimits(-1000.0, 1000.0);
    robot->AddLinkActuator(pushForward, "ROV", sf::I4());

    sf::Push* pushUp = new sf::Push("PushUp", false);
    pushUp->setForceLimits(-1000.0, 1000.0);
    robot->AddLinkActuator(pushUp, "ROV", sf::Transform(sf::Quaternion(0, M_PI_2, 0), sf::Vector3(0, 0, 0)));


    // -------------- ADD ROBOT --------------
    AddRobot(robot, sf::Transform(sf::IQ(), sf::Vector3(0.0, 0.0, 5.0))); // 5 m under ocean surface


    // -------------- TERRAIN --------------
    sf::Terrain* seabed = new sf::Terrain("Seabed", sf::GetDataPath() + "terrain.png", 1.0, 1.0, 5.0, "Rock", "seabed", 5.f);
    AddStaticEntity(seabed, sf::Transform(sf::IQ(), sf::Vector3(0,0,15.0)));
}