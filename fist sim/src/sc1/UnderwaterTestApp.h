#ifndef __Stonefish__UnderwaterTestApp__
#define __Stonefish__UnderwaterTestApp__

#include <core/GraphicalSimulationApp.h>
#include <graphics/OpenGLPrinter.h>
#include "UnderwaterTestManager.h"


class UnderwaterTestApp : public sf::GraphicalSimulationApp
{
public:
    UnderwaterTestApp(std::string dataDirPath, sf::RenderSettings s, sf::HelperSettings h, UnderwaterTestManager* sim);
    
    void DoHUD();
    void InitializeGUI();



    struct ControlInputs {
        bool fwd=false, back=false;   
        bool yawL=false, yawR=false;  
        bool up=false, down=false;    
        bool left=false, right=false; 
        bool pitchUp=false, pitchDown=false;
        bool turbo=false;             
    } in;

    // Cotnrol the ROV
    void KeyDown(SDL_Event* event) override;
    void KeyUp(SDL_Event* event) override;
    void applyVelocity();
    
private:
    sf::OpenGLPrinter* largePrint;
};

#endif
