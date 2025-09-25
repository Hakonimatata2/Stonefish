#include "UnderwaterTestApp.h"
#include "UnderwaterTestManager.h"
#include <cfenv>


int main(int argc, const char * argv[])
{
    sf::RenderSettings s;
    s.windowW = 1400;
    s.windowH = 800;
    s.aa = sf::RenderQuality::LOW;
    s.shadows = sf::RenderQuality::MEDIUM;
    s.ao = sf::RenderQuality::MEDIUM;
    s.atmosphere = sf::RenderQuality::MEDIUM;
    s.ocean = sf::RenderQuality::MEDIUM;
    s.ssr = sf::RenderQuality::MEDIUM;
    
    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = false;
    h.showBulletDebugInfo = false;
    h.showSensors = false;
    h.showActuators = false;
    h.showForces = false;
    
    UnderwaterTestManager simulationManager(200.0);
    simulationManager.setRealtimeFactor(1.0);
    UnderwaterTestApp app(std::string(DATA_DIR_PATH), s, h, &simulationManager);
    app.Run();
    
    return 0;
}

