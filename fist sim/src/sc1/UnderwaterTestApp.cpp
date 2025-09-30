#include "UnderwaterTestApp.h"

#include <actuators/Servo.h>
#include <actuators/Thruster.h>
#include <actuators/VariableBuoyancy.h>
#include <core/Robot.h>
#include <sensors/scalar/Accelerometer.h>
#include <sensors/scalar/IMU.h>
#include <sensors/scalar/DVL.h>
#include <sensors/vision/FLS.h>
#include <sensors/vision/SSS.h>
#include <graphics/IMGUI.h>
#include <utils/SystemUtil.hpp>
#include <comms/USBL.h>
#include <core/Console.h>
#include <Stonefish/actuators/Push.h>


UnderwaterTestApp::UnderwaterTestApp(std::string dataDirPath, sf::RenderSettings s, sf::HelperSettings h, UnderwaterTestManager* sim)
    : GraphicalSimulationApp("Underwater Test", dataDirPath, s, h, sim)
{
}

void UnderwaterTestApp::InitializeGUI()
{
    largePrint = new sf::OpenGLPrinter(sf::GetShaderPath() + std::string(STANDARD_FONT_NAME), 64.0);
}
void UnderwaterTestApp::DoHUD()
{
    GraphicalSimulationApp::DoHUD();
    applyVelocity();
}

// Tast ned
void UnderwaterTestApp::KeyDown(SDL_Event* event) {
    if (event && event->type == SDL_KEYDOWN) {
        switch (event->key.keysym.sym) {
            case SDLK_i:
                in.fwd = true; 
                break;
            case SDLK_k:
                in.back = true; 
                break;
            case SDLK_SPACE:
                in.up = true; 
                break;
            case SDLK_LSHIFT:
                in.down = true; 
                break;
            case SDLK_j:
                in.left = true;
                break;
            case SDLK_l:
                in.right = true;
                break;
            case SDLK_n:
                in.pitchUp = true;
                break;
            case SDLK_m:
                in.pitchDown = true;
                break;
            case SDLK_u:
                in.yawL = true;
                break;
            case SDLK_o:
                in.yawR = true;
                break;
            default: 
                break;
        }
    }
    sf::GraphicalSimulationApp::KeyDown(event); // bevar baseoppførsel
}

// Tast opp
void UnderwaterTestApp::KeyUp(SDL_Event* event) {
    if (event && event->type == SDL_KEYUP) {
        switch (event->key.keysym.sym) {
            case SDLK_i:
                in.fwd = false; 
                break;
            case SDLK_k:
                in.back = false; 
                break;
            case SDLK_SPACE:
                in.up = false; 
                break;
            case SDLK_LSHIFT:
                in.down = false; 
                break;
            case SDLK_j:
                in.left = false;
                break;
            case SDLK_l:
                in.right = false;
                break;
            case SDLK_n:
                in.pitchUp = false;
                break;
            case SDLK_m:
                in.pitchDown = false;
                break;
            case SDLK_u:
                in.yawL = false;
                break;
            case SDLK_o:
                in.yawR = false;
                break;
            default: 
                break;
        }
    }
    sf::GraphicalSimulationApp::KeyUp(event); // bevar baseoppførsel
}



void UnderwaterTestApp::applyVelocity() {
    // Get actuators
    sf::Push* pushForward = (sf::Push*)getSimulationManager()->getActuator("PushForward");
    sf::Push* pushUp = (sf::Push*)getSimulationManager()->getActuator("PushUp");

    float speed = 0.2f;

    // Get force based on input
    const sf::Scalar surge = speed * sf::Scalar((in.fwd?1:0) - (in.back?1:0)); // Fram  og tilbake
    const sf::Scalar heave = speed * sf::Scalar((in.up?1:0) - (in.down?1:0)); // opp og ned
    const sf::Scalar sway = speed * sf::Scalar((in.right?1:0) - (in.left?1:0)); // Høyre venstre
    const sf::Scalar pitch = 5*speed * sf::Scalar((in.pitchUp?1:0) - (in.pitchDown?1:0));
    const sf::Scalar yaw = 5*speed * sf::Scalar((in.yawL?1:0) - (in.yawR?1:0));
    
    // Apply forces
    // pushForward->setForce(surge);
    // pushUp->setForce(heave);

    auto* sim = getSimulationManager();
    auto* robot = sim->getRobot("Robot");
    if (robot) {
        auto* rb = robot->getBaseLink()->getRigidBody();
        if (!rb) return;

        sf::Vector3 world_lin_vel = sf::Vector3(surge, heave, sway);
        sf::Vector3 world_ang_vel = sf::Vector3(0, yaw, pitch);

        // Transformation from world to body
        auto rotation = robot->getBaseLink()->getCGTransform().getRotation();

        btVector3 body_lin_vel = quatRotate(rotation, world_lin_vel);
        btVector3 body_ang_vel = quatRotate(rotation, world_ang_vel);


        rb->setLinearVelocity(body_lin_vel);
        rb->setAngularVelocity(body_ang_vel);
    }
}