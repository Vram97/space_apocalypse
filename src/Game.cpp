#include "Game.h"
#include <iostream>
#include<ros/ros.h>

// Private functions
void Game::initVariables()
{
    this->points = 0;
    this->endGame = false;
    this->pub_game = nh_game.advertise<std_msgs::Float32MultiArray>("game_publisher", 25);

}

void Game::initTextures()
{
    this->textures["BULLET"] = new sf::Texture();
    this->textures["BULLET"]->loadFromFile("/home/shivaram/Code/sfml_tutorials/Textures/bullet.png");
}

void Game::initWindow()
{
    this->videoMode = sf::VideoMode(800, 600);
    this->window = new sf::RenderWindow(this->videoMode, "Game 3", sf::Style::Titlebar | sf::Style::Close);
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
    this->enemy=new Enemy(20.f,0.f);
}

Game::Game()
{
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

//Functions
void Game::run()
{
    while(this->window->isOpen())
    {   this->updatePollEvents();
        if(this->endGame == false)
        {
            this->update();
        }
        else
        {
            this->window->close();
        }
        
        this->pub_game.publish(ImageConverter::sfmlImageToROSImage(this->window->capture()));

        this->render();

        ros::spinOnce();
    }
}

void Game::initGUI()
{
    if(!this->font.loadFromFile("/usr/share/fonts/truetype/ubuntu/UbuntuMono-B.ttf"))
    {
        std::cout << "ERROR::GAME::INITGUI::Failed to load font" << "\n";
    }

    //Init point text
    this->pointText.setFont(this->font);
    this->pointText.setCharacterSize(24);
    this->pointText.setFillColor(sf::Color::White);
    this->pointText.setString("None");
    this->pointText.setPosition(sf::Vector2f(650.f, 10.f));

    this->gameOverText.setFont(this->font);
    this->gameOverText.setCharacterSize(60);
    this->gameOverText.setFillColor(sf::Color::Red);
    this->gameOverText.setString("Game Over!");
    
    //Init player GUI
    this->playerHpBar.setSize(sf::Vector2f(300.f, 25.f));
    this->playerHpBar.setFillColor(sf::Color::Red);
    this->playerHpBar.setPosition(sf::Vector2f(20.f, 20.f));

    this->playerHpBarBack = this->playerHpBar;
    this->playerHpBarBack.setFillColor(sf::Color(25, 25, 25, 200));
}

void Game::updatePollEvents()
{
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
    unsigned int counter = 0;
    while(!this->bullets.empty() && counter < this->bullets.size())
    {
        Bullet *bullet = this->bullets[counter];
        bullet->update();

        //Bullet culling (sides of screen)
        if(bullet->getBounds().top  < 0.f || bullet->getBounds().top>this->window->getSize().y)
        {
            //Delete bullet
            // delete this->bullets[counter];
            // this->bullets[counter]=nullptr;
            bullets.erase(bullets.begin() + counter);
            counter--;
        }
        else if(bullet->getBounds().left + bullet->getBounds().width > this->window->getSize().x)
        {
            //Delete bullet
            // delete this->bullets[counter];
            // this->bullets[counter]=nullptr;
            bullets.erase(bullets.begin() + counter);
            counter--;
        }
        else if(bullet->getBounds().left <0.f)
        {
            //Delete bullet
            // delete this->bullets[counter];
            // this->bullets[counter]=nullptr;
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
            if(enemy->getBounds().intersects(this->bullets[k]->getBounds()))
            {   
                if(enemy->getHp() <= this->bullets[k]->getDamage())
                {
                    this->points += 1;
                    // delete this->enemies[counter];
                    // delete this->bullets[k];
                    // this->bullets[k]=nullptr;
                    // this->enemies[counter]=nullptr;

                    this->enemies.erase(this->enemies.begin() + counter);
                    this->bullets.erase(this->bullets.begin() + k);
                    k--;
                    counter--;
                    enemy_removed=true;
                    break;
                }
                else
                {
                    enemy->takeDamage(this->bullets[k]->getDamage());
                    // delete this->bullets[k];
                    // this->bullets[k]=nullptr;
                    this->bullets.erase(this->bullets.begin() + k);
                    k--;
                    break;
                }
            }
            k++;
        }

        //Enemy culling (bottom of screen)
        if(!enemy_removed)
        {
            if(enemy->getBounds().top  + enemy->getBounds().height> this->window->getSize().y)
            {
                //Delete enemy
                // delete this->enemies[counter];
                this->enemies.erase(this->enemies.begin() + counter);
                enemy_removed=true;
                counter--;
            }
        }

        //Check if enemy collides with player
        if(enemy->getBounds().intersects(this->player->getBounds()))
        {
            this->player->loseHp(this->enemies[counter]->getDamage());
            // delete this->enemies[counter];
            // this->enemies[counter]=nullptr;8
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
        // std::cout<<"Bullet count: "<<this->bullets.size()<<std::endl;
    }
}

void Game::movePlayer(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    ROS_INFO("I heard: [%f]", msg->data[0]);
    ROS_INFO("I heard: [%f]", msg->data[1]);
    //Move player
    // this->player->move(std::min(0.01,(double)msg->data[0]),std::min(0.01,(double)msg->data[1]));
    this->player->move(msg->data[0],msg->data[1],this->window);
}

void Game::update()
{
    
    this->player->update(this->window);
    if(this->player->getHp() <= 0)
        this->endGame=true;
    
    this->updateEnemiesAndCombat();
    this->updateInput();
    this->updateGUI();
    this->updateBullets();

}

void Game::initWorld()
{
    if(!this->worldBackgroundTex.loadFromFile("/home/shivaram/Code/sfml_tutorials/Textures/space.jpg")){
        std::cout<<"ERROR::GAME::INITWORLD::Failed to load world background texture"<<std::endl;
    }
    this->worldBackground.setTexture(this->worldBackgroundTex);

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

void Game::updateGUI()
{
    std::stringstream ss;

    ss << "Points: " << this->points;

    this->pointText.setString(ss.str());

    //Update player GUI
    float hpPercent = static_cast<float>(this->player->getHp())/this->player->getHpMax();
    this->playerHpBar.setSize(sf::Vector2f(300.f * hpPercent, this->playerHpBar.getSize().y));


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
