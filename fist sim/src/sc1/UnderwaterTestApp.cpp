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
    applyThrusters();
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
            case SDLK_u:
                in.up = true; 
                break;
            case SDLK_o:
                in.down = true; 
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
            case SDLK_u:
                in.up = false; 
                break;
            case SDLK_o:
                in.down = false; 
                break;
            default: 
                break;
        }
    }
    sf::GraphicalSimulationApp::KeyUp(event); // bevar baseoppførsel
}



void UnderwaterTestApp::applyThrusters() {
    // Get actuators
    sf::Push* pushForward = (sf::Push*)getSimulationManager()->getActuator("PushForward");
    sf::Push* pushUp = (sf::Push*)getSimulationManager()->getActuator("PushUp");

    float gain = 1000.0f;

    // Get force based on input
    const sf::Scalar surge = gain * sf::Scalar((in.fwd?1:0) - (in.back?1:0));
    const sf::Scalar heave = gain * sf::Scalar((in.up?1:0) - (in.down?1:0));

    // Apply forces
    pushForward->setForce(surge);
    pushUp->setForce(heave);
 
}