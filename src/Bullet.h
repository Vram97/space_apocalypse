#pragma once

#ifndef BULLET_H
#define BULLET_H

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>


class  Bullet{
private:
    //Variables
    sf::Sprite shape;
    sf::Texture* texture;

    sf::Vector2f direction;
    float movementSpeed;
    float attackDamage;

public:
    Bullet(sf::Texture* texture, float pos_x, float pos_y, float dir_x, float dir_y, float movement_speed);
    virtual ~Bullet();

    // Accessors
    const sf::FloatRect getBounds() const;
    const int getDamage() const;

    //Functions
    void update();
    void render(sf::RenderTarget* target);
};

#endif