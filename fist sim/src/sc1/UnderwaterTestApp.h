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
    
private:
    sf::OpenGLPrinter* largePrint;
};

#endif
