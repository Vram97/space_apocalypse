#include <iostream>
#include "Player.h"

void Player::initVariables()
{
    this->movementSpeed = 300.f;
    this->attackCooldownMax = 10.f;
    this->attackCooldown = this->attackCooldownMax;
    this->direction = 1.0f;
    this->hpMax=10.0f;
    this->hp = this->hpMax;
}

void Player::initTexture()
{
    if(!this->texture.loadFromFile("/home/shivaram/Code/sfml_tutorials/Textures/shooter.png"))
    {
        std::cout << "ERROR::PLAYER::INITTEXTURE::Could not load texture file." << std::endl;
    }
    this->sprite.setTexture(this->texture);
}

void Player::initSprite()
{
    this->sprite.setPosition(0.f, 0.f);
}

Player::Player(float x, float y)
{   
    this->initVariables();
    this->initTexture();
    this->initSprite();
    
}

Player::~Player()
{
}

const sf::Vector2f &Player::getPos() const
{
    // TODO: insert return statement here
    return this->sprite.getPosition();
}

const sf::FloatRect Player::getBounds() const
{
    return this->sprite.getGlobalBounds();
}

const float &Player::getHp() const
{
    return this->hp;
}

const float &Player::getHpMax() const
{
    return this->hpMax;
}

const bool Player::canAttack()
{
    if(this->attackCooldown >= this->attackCooldownMax)
    {
        this->attackCooldown = 0.f;
        return true;
    }
    return false;
}

void Player::loseHp(int hp)
{
    this->hp-=hp;
    if(this->hp < 0)
        this->hp = 0;
}

void Player::move(const float dir_x, const float dir_y,sf::RenderTarget* target)
{
    // std::cout<<dir_x<<" "<<dir_y<<std::endl;
    if(dir_x<0){
        if(this->direction==1){
            this->direction = -1;
            this->sprite.setScale(this->direction, 1.0f);
            this->sprite.setOrigin(this->sprite.getGlobalBounds().width, 0.f);
        }
    }
    else if(dir_x>0){
        if(this->direction==-1){
            this->direction = 1;
            this->sprite.setScale(this->direction, 1.0f);
            this->sprite.setOrigin(0.f, 0.f);
        }
    }
    this->sprite.move(this->movementSpeed*dir_x, this->movementSpeed*dir_y);

    // Checking that the player is within bounds
    if(this->sprite.getPosition().x < 0.f)
        this->sprite.setPosition(0.f, this->sprite.getPosition().y);
    if(this->sprite.getPosition().x + this->getBounds().width > target->getSize().x)
        this->sprite.setPosition(target->getSize().x - this->getBounds().width, this->sprite.getPosition().y);
    if(this->sprite.getPosition().y < 0.f)
        this->sprite.setPosition(this->sprite.getPosition().x, 0.f);
    if(this->sprite.getPosition().y + this->getBounds().height > target->getSize().y)
        this->sprite.setPosition(this->sprite.getPosition().x, target->getSize().y - this->getBounds().height);
}

void Player::updateAttack()
{
    if(this->attackCooldown < this->attackCooldownMax)
        this->attackCooldown += 0.5f;
}

void Player::updateInput(sf::RenderTarget *target)
{
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)){
        if(this->direction==1){
            this->direction = -1;
            this->sprite.setScale(this->direction, 1.0f);
            this->sprite.setOrigin(this->sprite.getGlobalBounds().width, 0.f);
        }
        this->sprite.move(-1.f, 0.f);
    }

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)){
        if(this->direction==-1){
            this->direction = 1;
            this->sprite.setScale(this->direction, 1.0f);
            this->sprite.setOrigin(0.f, 0.f);
        }
        this->sprite.move(1.f, 0.f);
    }

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) 
        this->sprite.move(0.f, -1.f);
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S))
        this->sprite.move(0.f, 1.f);

    // Checking that the player is within bounds
    if(this->sprite.getPosition().x < 0.f)
        this->sprite.setPosition(0.f, this->sprite.getPosition().y);
    if(this->sprite.getPosition().x + this->getBounds().width > target->getSize().x)
        this->sprite.setPosition(target->getSize().x - this->getBounds().width, this->sprite.getPosition().y);
    if(this->sprite.getPosition().y < 0.f)
        this->sprite.setPosition(this->sprite.getPosition().x, 0.f);
    if(this->sprite.getPosition().y + this->getBounds().height > target->getSize().y)
        this->sprite.setPosition(this->sprite.getPosition().x, target->getSize().y - this->getBounds().height);
    
}

void Player::update(sf::RenderTarget *target)
{
    this->updateInput(target);
    this->updateAttack();
}

void Player::render(sf::RenderTarget *target)
{
    target->draw(this->sprite);
}
