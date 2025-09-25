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

UnderwaterTestManager::UnderwaterTestManager(sf::Scalar stepsPerSecond)
: SimulationManager(stepsPerSecond, sf::SolverType::SOLVER_SI, sf::CollisionFilteringType::COLLISION_EXCLUSIVE)
{
}

void UnderwaterTestManager::BuildScenario()
{
    ///////MATERIALS////////
    CreateMaterial("Fiberglass", sf::UnitSystem::Density(sf::CGS, sf::MKS, 1.0), 0.3);
    SetMaterialsInteraction("Fiberglass", "Fiberglass", 0.5, 0.2);
    
    ///////LOOKS///////////
    CreateLook("white", sf::Color::Gray(1.f), 0.9f, 0.0f, 0.f);

    EnableOcean(0.0);
    getOcean()->EnableCurrents();
    getAtmosphere()->SetSunPosition(0.0, 45.0);

    sf::BodyPhysicsSettings phy;
    phy.mode = sf::BodyPhysicsMode::SUBMERGED;
    phy.collisions = true;
    phy.buoyancy = true;

    auto vehicle_vis_obj = sf::GetDataPath() + "ResiFarmBlueRov.obj";
    // auto vehicle_vis_obj = sf::GetDataPath() + "sphere_R=1.obj";
    auto vehicle_phy_obj = sf::GetDataPath() + "sphere_R=1.obj";

    sf::Transform position = sf::Transform(sf::IQ(), sf::Vector3(0.0, -10.0, 0.0));

    sf::Polyhedron* vehicle = new sf::Polyhedron(
        "ROV",                 // navn
        phy,                   // BodyPhysicsSettings
        vehicle_vis_obj,
        0.01,                  // scale
        position,              // origin transform (identity matrix)
        vehicle_phy_obj,
        1.0,
        position,
        "Fiberglass",          // material
        "white"                // look
    );
    vehicle->isBuoyant();
    // vehicle->SetArbitraryPhysicalProperties(2.0, sf::Vector3(1.0, 0.5, 0.2), sf::Transform(sf::IQ(), sf::Vector3(0.2, 0.0, 0.0)));
    AddSolidEntity(vehicle, sf::I4());
}
