#include "Player.h"

#include <iostream>

void Player::initVariables()
{
    this->movementSpeed = 300.f;    // This is used by the move Player function that tracks the hand movement.
    this->attackCooldownMax = 10.f;
    this->attackCooldown = this->attackCooldownMax;  // This helps prevent the user from shooting continuously.
    this->direction = 1.0f;
    this->hpMax=10.0f;    // Full health of the player
    this->hp = this->hpMax;
    this->PACKAGE_PATH = ros::package::getPath("space_apocalypse");  // Path to the package.
}

void Player::initTexture()
{
    if(!this->texture.loadFromFile(this->PACKAGE_PATH + "/data/Textures/shooter.png"))
    {
        std::cout << "ERROR::PLAYER::INITTEXTURE::Could not load texture file." << std::endl;
    }
    this->sprite.setTexture(this->texture);
}

void Player::initSprite()
{
    this->sprite.setPosition(200.f, 200.f);   // Setting the starting position of the player.
}

/******************************************************************************
*                       PUBLIC METHOD DEFINITIONS                             *
*******************************************************************************/

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

// The function checks if the shoot button has been pressed for long.
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
    // This function reduces the health of the player. 
    // Called when the player collides with the falling debris.
    
    this->hp-=hp;
    if(this->hp < 0)
        this->hp = 0;
}

void Player::move(const float dir_x, const float dir_y,sf::RenderTarget* target)
{
    // This section of the code is used to flip the sprite when the player changes direction.
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
    // This function is used to update the attack cooldown.

    if(this->attackCooldown < this->attackCooldownMax)
        this->attackCooldown += 0.5f;
}

void Player::updateInput(sf::RenderTarget *target)
{
    // This function is used to move the player using the keyboard.

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
    // Updating all Player variables.
    this->updateInput(target);
    this->updateAttack();
}

void Player::render(sf::RenderTarget *target)
{
    target->draw(this->sprite);
}
