#pragma once

#include <SFML/Graphics.hpp>
#include <sensor_msgs/Image.h>

class ImageConverter{
    /* This class can be used to write methods that convert the SFML images to various formats 
       as  required by the game.*/
    public:
        static sensor_msgs::Image sfmlImageToROSImage(const sf::Image& sfmlImage);

        // TODO: Add method to convert SFML image to OpenCV image.
};