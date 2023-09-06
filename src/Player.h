#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>

class Player{
private:
    //Variables
    // sf::RectangleShape shape;
    float movementSpeed;

    sf::Sprite sprite;
    sf::Texture texture;
    float attackCooldown;
    float attackCooldownMax;
    float hp;
    float hpMax;

    //Private Functions
    void initVariables();
    void initTexture();
    void initSprite();
    // void initShape();

public:
    Player(float x = 0.f, float y = 0.f);
    virtual ~Player();

    //Variables
    float direction;

    //Accessors
    const sf::Vector2f& getPos() const;
    const sf::FloatRect getBounds() const;
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