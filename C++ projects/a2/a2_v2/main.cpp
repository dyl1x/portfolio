/*! @file
 *
 *  @brief Main entry point for assignment 2.
 *  @details Creats an instance of the simulator depending on the input at execution.
 *  The game mode will be decided based on the input and the simulator will be passed into the
 *  appropriate pilot class which will control the friendly aircraft in the simulation, to locate and intercept bogies.
 *
 *  Basic mode - Bogies are stationary
 *  Advanced - Bogies move in a straight line at contant velocity.
 *
 *  @author Chamath Edirisinhege - 12977866
 *  @date 23 Sept 2021
*/

#include <iostream>
#include "simulator.h"
#include "bscpilot.h"
#include "advpilot.h"

using simulator::GameMode;

int main(int argc, char *argv[])
{

    GameMode game_mode = GameMode::ADVANCED;
    //If code is started with --advanced, it will run in advanced mode
    std::cout << "Run with: " << argv[0] << " --advanced to run in advanced mode" << std::endl;
    if(argc>1){
        if(strcmp (argv[1],"-advanced")){
            std::cout << "Advanced Mode Activated" << std::endl;
            game_mode = GameMode::ADVANCED;
        }
    }
    std::shared_ptr<simulator::Simulator> sim(new simulator::Simulator(true,game_mode));

    switch (game_mode) {
    case GameMode::BASIC: {//do basic mode stuff
        BscPilot pilot(sim);
        pilot.start();
        break;
    }
    case GameMode::ADVANCED: {//do advanced mode stuff
        AdvPilot pilot(sim);
        pilot.start();
        break;
    }
    }

    return 0;
}
