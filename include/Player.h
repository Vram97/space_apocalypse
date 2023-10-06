#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>

class Player{
private:

    // Animation
    sf::Sprite sprite;
    sf::Texture texture;
    float attackCooldown;    // Prevents continuous shooting.
    float attackCooldownMax;
    float hp;
    float hpMax;
    float movementSpeed;

    std::string PACKAGE_PATH;

    //Private Functions
    void initVariables();
    void initTexture();
    void initSprite();

public:
    Player(float x = 0.f, float y = 0.f);  // Constructor
    virtual ~Player();                     // Destructor

    //Variables
    float direction;

    //Accessors
    const sf::Vector2f& getPos() const;
    const sf::FloatRect getBounds() const;  // Returns the bounding box of the sprite.
    const float& getHp() const;
    const float& getHpMax() const;

    const bool canAttack();

    //Modifiers
    void loseHp(int hp);

    //Functions
    void move(const float dir_x, const float dir_y,sf::RenderTarget* target);
    void updateAttack();
    void updateInput(sf::RenderTarget* target);
    void update(sf::RenderTarget* target);
    void render(sf::RenderTarget* target);
};