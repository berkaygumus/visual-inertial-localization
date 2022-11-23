#pragma once

#include <arp/Autopilot.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <SDL2/SDL.h>

namespace gui {

class Renderer {
 private:
    SDL_Event event;
    SDL_Window * window;
    SDL_Renderer * renderer;
    SDL_Texture * texture;
 public:
    /// @brief Setup rendering.
    Renderer();
    /// @brief Clean up rendering.
    ~Renderer();
    /// @brief Display the image with the respective overlay.
    /// @param image Image to be displayed by the renderer.
    /// @param droneStatus Status of the drone for the overlay.
    /// @param batteryLevel Battery level of the drone for the overlay.
    void render(cv::Mat& image, arp::Autopilot::DroneStatus droneStatus, float batteryLevel);
    /// @brief Returns true if SDL_Window is closed (SDL_QUIT event). 
    /// @return 
    bool checkQuit();
};

}