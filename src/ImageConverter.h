#pragma once

#include <SFML/Graphics.hpp>
#include <sensor_msgs/Image.h>

class ImageConverter{
    public:
        static sensor_msgs::Image sfmlImageToROSImage(const sf::Image& sfmlImage);
};