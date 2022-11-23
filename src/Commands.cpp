#include <Commands.hpp>

#include <SDL2/SDL.h>

namespace Commands {

// Prints the command to be executed + drone status
void printCommand(const std::string& text, const arp::Autopilot::DroneStatus droneStatus)
{
    std::cout << text << droneStatus;
}

// Prints whether the command was successful or not
void printStatus(const bool success)
{
    if(success) {
        std::cout << " [ OK ]" << std::endl;
    } else {
        std::cout << " [FAIL]" << std::endl;
    }
}

void checkKeysForCommand(arp::Autopilot& autopilot)
{
    //Multiple Key Capture Begins
    const Uint8 *state = SDL_GetKeyboardState(NULL);

    // check states!
    // command
    auto droneStatus = autopilot.droneStatus();

    if (state[SDL_SCANCODE_ESCAPE]) {
      printCommand("ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=", droneStatus);
      bool success = autopilot.estopReset();
      printStatus(success);
    }

    if (state[SDL_SCANCODE_T]) {
      printCommand("Taking off...                          status=", droneStatus);
      bool success = autopilot.takeoff();
      printStatus(success);
    }

    if (state[SDL_SCANCODE_L]) {
      printCommand("Going to land...                       status=", droneStatus);
      bool success = autopilot.land();
      printStatus(success);
    }
    if (state[SDL_SCANCODE_C]) {
      printCommand("Requesting flattrim calibration...     status=", droneStatus);
      bool success = autopilot.flattrimCalibrate();
      printStatus(success);
    }

    if (droneStatus == autopilot.Flying || droneStatus == autopilot.Hovering || droneStatus == autopilot.Flying2) {
        // Compute manual move values.
        double forward = state[SDL_SCANCODE_UP]   - state[SDL_SCANCODE_DOWN];
        double left    = state[SDL_SCANCODE_LEFT] - state[SDL_SCANCODE_RIGHT];
        double up      = state[SDL_SCANCODE_W]    - state[SDL_SCANCODE_S];
        double yawLeft = state[SDL_SCANCODE_A]    - state[SDL_SCANCODE_D];
        bool success = autopilot.manualMove(forward, left, up, yawLeft);
        
        // Only print non-zero move commands.
        if (forward != 0 || left != 0 || up != 0 || yawLeft != 0) {
            // printCommand("Sending move command...               status=", droneStatus);
            printCommand("Sending twist {u=" + std::to_string(static_cast<int>(up)) + 
                        ", f=" + std::to_string(static_cast<int>(forward)) +
                        ", l=" + std::to_string(static_cast<int>(left)) +
                        ", yL=" + std::to_string(static_cast<int>(yawLeft)) +
                        "}... status=", droneStatus);
            printStatus(success);
        } 
    }
}
}