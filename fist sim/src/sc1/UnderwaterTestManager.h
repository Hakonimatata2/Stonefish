#ifndef __Stonefish__UnderwaterTestManager__
#define __Stonefish__UnderwaterTestManager__

#include <core/SimulationManager.h>

//#define PARSED_SCENARIO

class UnderwaterTestManager : public sf::SimulationManager
{
public:
    UnderwaterTestManager(sf::Scalar stepsPerSecond);
    
    void BuildScenario();
};

#endif
