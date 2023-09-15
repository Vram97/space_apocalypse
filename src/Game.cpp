#include "Game.h"

#include <ros/ros.h>
#include <iostream>


// Private functions
void Game::initVariables()
{
    this->points = 0;
    this->endGame = false;  // Game ends when player loses all health.
    this->PACKAGE_PATH = ros::package::getPath("space_apocalypse");  // Path to the package.
    this->config = YAML::LoadFile(this->PACKAGE_PATH + "/config/config.yaml");  // Loading the config file.
    this->pub_game = nh_game.advertise<std_msgs::Float32MultiArray>("game_publisher", this->config["FrameRate"].as<int>()); // Publisher for the game. 
    this->sub_move_player = nh_move_player.subscribe("hand_tracker_publisher", this->config["FrameRate"].as<int>(), &Game::movePlayer, this);  // Subscriber for the hand tracker. The default rate is 25 Hz because my hand tracker publishes at 25 Hz.

}

void Game::initTextures()
{
    this->textures["BULLET"] = new sf::Texture();
    this->textures["BULLET"]->loadFromFile(this->PACKAGE_PATH + "/data/Textures/bullet.png");
}

void Game::initWindow()
{
    this->videoMode = sf::VideoMode(800, 600);  // This is where the size of the window is set.
    this->window = new sf::RenderWindow(this->videoMode, "Space Apocalypse", sf::Style::Titlebar | sf::Style::Close);
    this->window->setFramerateLimit(60);
    this->window->setVerticalSyncEnabled(false);

}

void Game::initEnemies()
{
    this->spawnTimerMax=10.f;
    this->spawnTimer = this->spawnTimerMax;
}

void Game::initPlayer()
{
    this->player = new Player();
    this->enemy=new Enemy(20.f,0.f); // Initializing the enemy object.
}

void Game::initGUI()
{   
    // Loading font from the default location in Ubuntu.
    if(!this->font.loadFromFile("/usr/share/fonts/truetype/ubuntu/UbuntuMono-B.ttf"))
    {
        std::cout << "ERROR::GAME::INITGUI::Failed to load font" << "\n";
    }

    // Initializing the text for the points.
    this->pointText.setFont(this->font);
    this->pointText.setCharacterSize(24);
    this->pointText.setFillColor(sf::Color::White);
    this->pointText.setString("None");
    this->pointText.setPosition(sf::Vector2f(650.f, 10.f));

    // Initializing the text for the game over message.
    this->gameOverText.setFont(this->font);
    this->gameOverText.setCharacterSize(60);
    this->gameOverText.setFillColor(sf::Color::Red);
    this->gameOverText.setString("Game Over!");
    
    // Initializing the player health bar.
    this->playerHpBar.setSize(sf::Vector2f(300.f, 25.f));
    this->playerHpBar.setFillColor(sf::Color::Red);
    this->playerHpBar.setPosition(sf::Vector2f(20.f, 20.f));
    this->playerHpBarBack = this->playerHpBar;
    this->playerHpBarBack.setFillColor(sf::Color(25, 25, 25, 200));
}

void Game::initWorld()
{   // Initializing the background image for the game.
    if(!this->worldBackgroundTex.loadFromFile(this->PACKAGE_PATH + "/data/Textures/space.jpg")){
        std::cout<<"ERROR::GAME::INITWORLD::Failed to load world background texture"<<std::endl;
    }
    this->worldBackground.setTexture(this->worldBackgroundTex);

}
/******************************************************************************
*                       PUBLIC METHOD DEFINITIONS                             *                       *
*******************************************************************************/

Game::Game()
{   // Constructor that initializes all the variables required by the game.

    this->initVariables();
    this->initWindow();
    this->initPlayer();
    this->initEnemies();
    this->initTextures();
    this->initGUI();
    this->initWorld();
}

Game::~Game()
{
    delete this->window;
    delete this->player;

    //Delete textures
    for(auto &i : this->textures)
    {
        delete i.second;
    }

    //Delete bullets
    for(auto *i : this->bullets)
    {
        delete i;
    }

    //Delete enemies
    for(auto *i : this->enemies)
    {
        delete i;
    }
}


void Game::run()
{
    while(this->window->isOpen())
    {   
        this->updatePollEvents();
        if(this->endGame == false)
        {
            this->update();
        }
        else
        {
            this->window->close();  // Close the window when the game ends.
        }

        // This is where the game publisher publishes the game image as a ROS message.
        this->pub_game.publish(ImageConverter::sfmlImageToROSImage(this->window->capture()));

        this->render();

        ros::spinOnce();
    }
}

void Game::updatePollEvents()
{
    // Event handling for Esc and Closr.
    sf::Event event;

    while(this->window->pollEvent(event))
    {
        if(event.type == sf::Event::Closed)
            this->window->close();
        if(event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape)
            this->window->close();
    }
}

void Game::updateBullets()
{   
    // Updating the movement of bullets and deleting them when they strike or go out of bounds.
    unsigned int counter = 0;
    while(!this->bullets.empty() && counter < this->bullets.size())
    {
        Bullet *bullet = this->bullets[counter];
        bullet->update();

        //Bullet culling (Top of screen)
        if(bullet->getBounds().top  < 0.f || bullet->getBounds().top>this->window->getSize().y)
        {

            bullets.erase(bullets.begin() + counter);
            counter--;
        }
        else if(bullet->getBounds().left + bullet->getBounds().width > this->window->getSize().x)
        {
            // Bullet culling right of screen.
            bullets.erase(bullets.begin() + counter);
            counter--;
        }
        else if(bullet->getBounds().left <0.f)
        {
            // Bullet culling left of screen.
            bullets.erase(bullets.begin() + counter);
            counter--;
        }
        else{
            ;
        }
        counter++;
    }
}

void Game::updateEnemiesAndCombat()
{
    //Spawning
    this->spawnTimer += 0.5f;
    if(this->spawnTimer >= this->spawnTimerMax)
    {
        this->enemies.push_back(new Enemy(rand()%this->window->getSize().x, 0));
        this->spawnTimer = 0.f;
    }

    //Update
    unsigned int counter = 0;
    while(!this->enemies.empty() && counter < this->enemies.size())
    {
        Enemy *enemy = this->enemies[counter];
        enemy->update();

        bool enemy_removed=false;

        //Enemy player collision
        unsigned int k=0;
        while(k < this->bullets.size())
        {   
            // When the bullet strikes the debris.
            if(enemy->getBounds().intersects(this->bullets[k]->getBounds()))
            {   
                // When the debris is destroyed.
                if(enemy->getHp() <= this->bullets[k]->getDamage())
                {
                    this->points += 1;

                    this->enemies.erase(this->enemies.begin() + counter);
                    this->bullets.erase(this->bullets.begin() + k);
                    k--;
                    counter--;
                    enemy_removed=true;
                    break;
                }
                else
                {   // When the debris is not destroyed.
                    enemy->takeDamage(this->bullets[k]->getDamage());
                    this->bullets.erase(this->bullets.begin() + k);
                    k--;
                    break;
                }
            }
            k++;
        }

        // Enemy culling (bottom of screen)
        if(!enemy_removed)
        {
            if(enemy->getBounds().top  + enemy->getBounds().height> this->window->getSize().y)
            {
                //Delete enemy
                this->enemies.erase(this->enemies.begin() + counter);
                enemy_removed=true;
                counter--;
            }
        }

        //Check if enemy collides with player
        if(enemy->getBounds().intersects(this->player->getBounds()))
        {
            this->player->loseHp(this->enemies[counter]->getDamage());
            this->enemies.erase(this->enemies.begin() + counter);
            counter--;
        }

        counter++;
    }
}

void Game::updateInput()
{
    // Shoot bullets
    if(sf::Mouse::isButtonPressed(sf::Mouse::Left) && this->player->canAttack()){
        this->bullets.push_back(new Bullet(this->textures["BULLET"], this->player->getPos().x + this->player->direction * this->player->getBounds().width/2, this->player->getPos().y + 30, this->player->direction, 0.f, 5.f));
    }
}

void Game::movePlayer(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //Move player
    this->player->move(msg->data[0],msg->data[1],this->window);
}

void Game::updateGUI()
{   // This function updates the Points and Player Health Bar.
    std::stringstream ss;

    ss << "Points: " << this->points;

    this->pointText.setString(ss.str());

    //Update player GUI
    float hpPercent = static_cast<float>(this->player->getHp())/this->player->getHpMax();
    this->playerHpBar.setSize(sf::Vector2f(300.f * hpPercent, this->playerHpBar.getSize().y));

}

void Game::update()
{   // This function updates the game objects.
    
    this->player->update(this->window);

    // Checking if the player has lost all health.
    if(this->player->getHp() <= 0)
        this->endGame=true;
    
    this->updateEnemiesAndCombat();
    this->updateInput();
    this->updateGUI();
    this->updateBullets();

}

void Game::renderGUI()
{
    this->window->draw(this->pointText);
    this->window->draw(this->playerHpBarBack);
    this->window->draw(this->playerHpBar);
}

void Game::renderWorld()
{
    this->window->draw(this->worldBackground);
}

void Game::render()
{
    this->window->clear();

    //Draw world
    this->renderWorld();

    //Draw game objects
    this->player->render(this->window);

    for(auto *i : this->bullets)
    {
        i->render(this->window);
    }

    // Render enemies
    for(auto *i : this->enemies)
    {
        i->render(this->window);
    }

    //Render GUI
    this->renderGUI();

    //Draw mouse cursor
    if(this->player->getHp() == 0)
        this->window->draw(this->gameOverText);

    //Draw game
    this->window->display();
}
