#include "Enemy.h"

void Enemy::initShape()
{
    this->shape.setRadius(this->pointCount*2);
    this->shape.setFillColor(sf::Color(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1));
    this->shape.setPointCount(this->pointCount);
}

void Enemy::initVariables(){
    this->type = 0;
    this->hpMax = 5;
    this->hp = this->hpMax;
    this->pointCount = rand() % 8 + 3;
    this->speed= static_cast<float>(this->pointCount/2);
}

Enemy::Enemy(float pos_x, float pos_y)
{
    this->initVariables();
    this->initShape();
    this->shape.setPosition(pos_x, pos_y);
}

Enemy::~Enemy()
{


}

const sf::FloatRect Enemy::getBounds() const
{
    return this->shape.getGlobalBounds();
}

const int Enemy::getHp() const
{
    
    return this->hp;
}

const int Enemy::getDamage() const
{
    return this->pointCount/10;
}

void Enemy::takeDamage(int damage)
{
    this->hp -= damage;
}

// Functions
void Enemy::update()
{

    this->shape.move(0.f, this->speed);

}

void Enemy::render(sf::RenderTarget *target)
{
    target->draw(this->shape);

}
