#include "ImageConverter.h"

sensor_msgs::Image ImageConverter::sfmlImageToROSImage(const sf::Image& sfmlImage) {
    sensor_msgs::Image rosImage;

    // Fill in the header
    rosImage.header.stamp = ros::Time::now();
    rosImage.header.frame_id = "window_frame"; // Set your desired frame ID

    // Get the dimensions from the SFML image
    int width = sfmlImage.getSize().x;
    int height = sfmlImage.getSize().y;
    rosImage.height = height;
    rosImage.width = width;

    // Define the number of channels and encoding type
    rosImage.encoding = "rgb8"; // Set the encoding type (e.g., "rgb8" for RGB images)
    rosImage.is_bigendian = false;
    rosImage.step = width * 3; // Assuming 3 channels (RGB)

    // Create an array for the pixel data
    size_t dataSize = width * height * 3; // 3 channels (RGB)
    rosImage.data.resize(dataSize);

    // Copy pixel data from SFML image to ROS image
    const sf::Uint8* sfmlPixels = sfmlImage.getPixelsPtr();
    memcpy(&rosImage.data[0], sfmlPixels, dataSize);

    return rosImage;
}