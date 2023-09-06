#pragma once

#include<SFML/Graphics.hpp>

class Enemy{
private:
    sf::CircleShape shape;

    int type;
    int pointCount;
    int hp;
    int speed;
    int hpMax;

    void initVariables();
    void initShape();
    void initTexture();

    

public:
    Enemy(float pos_x,float pos_y);
    virtual ~Enemy();

    // Accessors
    const sf::FloatRect getBounds() const;
    const int getHp() const;
    const int getDamage() const;

    void takeDamage(int damage);
    void update();
    void render(sf::RenderTarget* target);

};