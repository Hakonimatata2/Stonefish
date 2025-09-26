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

UnderwaterTestManager::UnderwaterTestManager(sf::Scalar stepsPerSecond)
: SimulationManager(stepsPerSecond, sf::SolverType::SOLVER_SI, sf::CollisionFilteringType::COLLISION_EXCLUSIVE)
{
}

void UnderwaterTestManager::BuildScenario()
{
    // -------------- MATERIALS--------------
    CreateMaterial("Fiberglass", sf::UnitSystem::Density(sf::CGS, sf::MKS, 1.0), 0.3);
    CreateMaterial("Dummy", sf::UnitSystem::Density(sf::CGS, sf::MKS, 1.0), 0.3);
    SetMaterialsInteraction("Fiberglass", "Fiberglass", 0.5, 0.2);
    
    // -------------- LOOKS--------------
    CreateLook("white", sf::Color::Gray(1.f), 0.9f, 0.0f, 0.f);


    // -------------- OCEAN--------------
    EnableOcean(0.0);
    getOcean()->EnableCurrents();
    getAtmosphere()->SetSunPosition(0.0, 45.0);

    // -------------- Physics--------------
    sf::BodyPhysicsSettings phy;
    phy.mode = sf::BodyPhysicsMode::SUBMERGED;
    phy.collisions = true;
    phy.buoyancy = true;
    

    // -------------- Positions--------------
    
    //        I BLENDER             x         z       -y
    auto to_ref     = sf::Vector3(-0.1178, -0.6932, 0.3703);
    auto to_dvl     = sf::Vector3(-0.2883, -0.6684, 0.2624);
    auto to_mbs     = sf::Vector3(-0.2883, -0.6599, 0.4908);
    auto to_ping    = sf::Vector3(-0.4100, -0.4906, 0.5206);
    
    auto rel_mbs = to_ref - to_mbs;
    auto rel_ping = to_ref - to_ping;
    
    
    // -------------- DEFINING THE VEHICLE --------------

    sf::Polyhedron* vehicle = new sf::Polyhedron(
        "ROV",                                          // navn
        phy,                                            // BodyPhysicsSettings
        sf::GetDataPath() + "ResiFarmBlueRov.obj",      // Vsible object
        0.01,                                           // scale
        sf::Transform(sf::IQ(), to_ref),                // origin transform (identity matrix)
        sf::GetDataPath() + "sphere_R=1.obj",           // Physical object
        1.0,
        sf::I4(),
        "Fiberglass",          // material
        "white"                // look
    );
    sf::Robot* robot = new sf::GeneralRobot("Robot", false);
    std::vector<sf::SolidEntity*> links; 
    robot->DefineLinks(vehicle, links);
    robot->BuildKinematicStructure();


    // -------------- SENSORS --------------

    sf::Multibeam* mb = new sf::Multibeam("Multibeam", 120.0, 128, 1.0, 1);
    mb->setRange(0.5, 50.0);
    mb->setNoise(0.1);
    robot->AddLinkSensor(mb, "ROV", sf::Transform(sf::Quaternion(M_PI_2, 0, 0), rel_mbs));

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


    // -------------- ADD ROBOT --------------
    AddRobot(robot, sf::Transform(sf::IQ(), sf::Vector3(0.0, 0.0, 0.0)));
}
