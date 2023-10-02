#include "Bullet.h"

/******************************************************************************
*                       PUBLIC METHOD DEFINITIONS                             *
*******************************************************************************/

Bullet::Bullet(sf::Texture *texture, float pos_x, float pos_y, float dir_x, float dir_y, float movement_speed)
{
    this->shape.setTexture(*texture);
    this->shape.setPosition(pos_x, pos_y);

    // The directions are determined by the pose of the player.
    this->direction.x = dir_x;
    this->direction.y = dir_y;
    this->movementSpeed = movement_speed;    
    this->attackDamage = 5;               
}

Bullet::~Bullet()
{
    delete this->texture;
}

const sf::FloatRect Bullet::getBounds() const
{   // This function returns the bounding box of the bullet sprite.

    return this->shape.getGlobalBounds();
}

const int Bullet::getDamage() const
{
    return this->attackDamage;
}

void Bullet::update()
{  // This function updates the position of the bullet.
    this->shape.move(this->movementSpeed*this->direction);
}

void Bullet::render(sf::RenderTarget *target)
{
    target->draw(this->shape);
}
