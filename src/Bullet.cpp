#include "Bullet.h"

Bullet::Bullet(sf::Texture *texture, float pos_x, float pos_y, float dir_x, float dir_y, float movement_speed)
{
    this->shape.setTexture(*texture);
    this->shape.setPosition(pos_x, pos_y);
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
{
    return this->shape.getGlobalBounds();
}

const int Bullet::getDamage() const
{
    return this->attackDamage;
}

void Bullet::update()
{
    //Moveaaaa
    this->shape.move(this->movementSpeed*this->direction);
}

void Bullet::render(sf::RenderTarget *target)
{
    target->draw(this->shape);
}
