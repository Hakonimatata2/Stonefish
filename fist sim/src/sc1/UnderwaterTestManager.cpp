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
#include <Stonefish/actuators/Light.h>

UnderwaterTestManager::UnderwaterTestManager(sf::Scalar stepsPerSecond)
: SimulationManager(stepsPerSecond, sf::SolverType::SOLVER_SI, sf::CollisionFilteringType::COLLISION_EXCLUSIVE)
{
}

void UnderwaterTestManager::BuildScenario()
{
    // -------------- MATERIALS--------------
    CreateMaterial("Rock", sf::UnitSystem::Density(sf::CGS, sf::MKS, 3.0), 0.6);
    
    // -------------- LOOKS --------------
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
    

    // -------------- Robot --------------
    BuildRobot(sf::Vector3(0, 0, -2), phy);

    // -------------- TERRAIN --------------
    sf::Terrain* seabed = new sf::Terrain(
        "Seabed",                           // name 
        sf::GetDataPath() + "terrain.png",  // height map
        1.0,                                // scale x
        1.0,                                // scale y
        5.0,                                // height
        "Rock",                             // material
        "seabed",                           // look
        5.f                                 // uv scale (scale of texture coordinates)
    );
    AddStaticEntity(seabed, sf::Transform(sf::IQ(), sf::Vector3(0,0,15.0)));
}

void UnderwaterTestManager::BuildRobot(sf::Vector3 position, sf::BodyPhysicsSettings &phy)
{
    // UUV look and texture
    CreateLook(
        "blueRovTexture",
        sf::Color::Gray(1.f),                       // color tint
        0.1f, 0.0f, 0.0f,                           // roughness, metalness, reflectivity
        sf::GetDataPath() + "BlueROV/BlueRov.png",  // ALBEDO texture
        ""                                          // NORMAL texture
    );

    CreateLook("white", sf::Color::Gray(1.f), 0.9f, 0.0f, 0.f);
    CreateMaterial("ROV_Material", sf::UnitSystem::Density(sf::CGS, sf::MKS, 1.0), 0.3);
    CreateMaterial("Neutral", sf::UnitSystem::Density(sf::CGS, sf::MKS, 1.0), 0.3);

    // -------------- Positions --------------
    // From Blender Coordinates to NED: [x, y, z] -> [-x, -y, -z]
    auto toMBS      = - sf::Vector3(-0.147759, 0.0, 0.225529);
    auto toPing     = - sf::Vector3(0.024241, 0.12, 0.255529);
    auto toCamera   = - sf::Vector3(0.13, 0, 0.153);
    
    // -------------- DEFINING THE VEHICLE --------------
    phy.buoyancy = false;
    sf::Polyhedron* vehicle = new sf::Polyhedron(
        "Vehicle",                                                      // navn
        phy,                                                            // BodyPhysicsSettings
        // sf::GetDataPath() + "BlueROV/ROV_with_uv.obj",                  // Vsible object
        sf::GetDataPath() + "BlueROV/ResiFarmBlueRov-simplified.obj",   // Vsible object
        1.0,                                                            // scale
        sf::I4(),                                                       // origin transform
        sf::GetDataPath() + "BlueROV/ResiFarmBlueRov-simplified.obj",   // Physical object. Determines drag etc.
        1.0,                                                            // scale
        sf::I4(),                                                       // trasform
        "ROV_Material",                                                 // material
        "blueRovTexture",                                               // look
        0.005                                                           // thickness
    );
    sf::Compound* uuv = new sf::Compound("UUV", phy, vehicle, sf::I4());


    sf::Scalar mass = 14.7;
    sf::Scalar rhoWater = 1000.0; // kg/m^3, 1025.0 sea water
    sf::Scalar initialVol = uuv->getVolume();
    sf::Scalar desiredVolume = mass / rhoWater;
    sf::Scalar volumeToAdd = desiredVolume - initialVol;
    sf::Scalar sideLength = std::cbrt(volumeToAdd); // cube root
    
    // Add volume
    phy.buoyancy = true;
    sf::Box* box = new sf::Box(
        "box", 
        phy, 
        sf::Vector3(sideLength, sideLength, sideLength), 
        sf::Transform(sf::IQ(), sf::Vector3(0, 0, 0)), 
        "Neutral", 
        "white"
    );
    uuv->AddInternalPart(box, sf::Transform(sf::IQ(), sf::Vector3(0.0, 0.0, 0.0)));
    uuv->setDisplayInternalParts(false);

    // Set correct inertia and CG
    uuv->SetArbitraryPhysicalProperties(
        mass,                                           // mass
        sf::Vector3(1, 1, 1),                           // Ixx, Iyy, Izz
        sf::Transform(sf::IQ(), sf::Vector3(0, 0, 0))   // CG
    );

    // Define vehicle as a single link robot
    sf::Robot* robot = new sf::GeneralRobot("Robot", false);
    std::vector<sf::SolidEntity*> links; 
    robot->DefineLinks(uuv, links);
    robot->BuildKinematicStructure();

    // -------------- SENSORS --------------

    // Multibeam sonar / forward looking sonar
    sf::FLS* fls = new sf::FLS("FLS", 512, 500, 120.0, 30.0, 0.5, 10.0, sf::ColorMap::HOT);
    fls->setGain(1.1);
    fls->setNoise(0.01, 0.02);
    fls->setDisplayOnScreen(true, 900, 250, 0.4f);
    robot->AddVisionSensor(fls, "UUV", sf::Transform(sf::Quaternion(M_PI_2, 0, M_PI_2), toMBS));

    // Ping 360
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
    robot->AddVisionSensor(msis, "UUV", sf::Transform(sf::Quaternion(0, 0, M_PI_2), toPing));

    // Camera
    sf::ColorCamera* cam = new sf::ColorCamera(
        "Cam",  // name
        800,    // res x
        600,    // res y
        60.0,   // horizontal fov
        10.0    // fps
    );
    // robot->AddVisionSensor(cam, "UUV", sf::I4());

    // Lights 
    auto toLight1   = sf::Transform(sf::IQ(), -sf::Vector3(0.13, 0, 0.153));
    sf::Light* l1 = new sf::Light("Spot", 0.1, 30.0, sf::Color::BlackBody(5600.0), 2000.0);
    // robot->AddLinkActuator(l1, "UUV", toLight1);


    // -------------- ACTUATORS --------------
    sf::Transform to_cg_transform = uuv->getCGTransform();
    sf::Vector3 to_cg_vec = to_cg_transform.getOrigin(); 

    sf::Push* pushForward = new sf::Push("PushForward", false);
    pushForward->setForceLimits(-1000.0, 1000.0);
    robot->AddLinkActuator(pushForward, "UUV", sf::Transform(sf::IQ(), to_cg_vec));

    sf::Push* pushUp = new sf::Push("PushUp", false);
    pushUp->setForceLimits(-1000.0, 1000.0);
    robot->AddLinkActuator(pushUp, "UUV", sf::Transform(sf::Quaternion(0, M_PI_2, 0), to_cg_vec));


    // -------------- ADD ROBOT --------------
    AddRobot(robot, sf::Transform(sf::IQ(), sf::Vector3(0.0, 0.0, -1.0)));

}