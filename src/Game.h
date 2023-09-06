#pragma once

#include "Player.h"
#include "Bullet.h"
#include "Enemy.h"

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <unordered_map>
#include <sstream>
#include <std_msgs/Float32MultiArray.h>



class Game{
private:
    //Variables
    sf::RenderWindow* window;
    sf::Event sfEvent;
    sf::VideoMode videoMode;
    sf::Clock dtClock;
    int points;
    bool endGame;


    //GUI
    sf::Font font;
    sf::Text pointText;

    sf::Text gameOverText;

    // Player GUI
    sf::RectangleShape playerHpBar;
    sf::RectangleShape playerHpBarBack;

    // World
    sf::Texture worldBackgroundTex;
    sf::Sprite worldBackground;

    // Resources
    std::unordered_map<std::string, sf::Texture*> textures;
    std::vector<Bullet*> bullets;

    // Game objects
    Player* player;

    // Enemies
    std::vector<Enemy*> enemies;
    Enemy* enemy;
    float spawnTimer;
    float spawnTimerMax;

    //Private functions
    void initVariables();
    void initTextures();
    void initWindow();
    void initEnemies();
    void initPlayer();
    void initGUI();
    void initWorld();
    

public:
    Game();
    virtual ~Game();
    void run();

    void updatePollEvents();
    void updateBullets();
    void updateEnemiesAndCombat();
    void updateInput();
    void movePlayer(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void updateGUI();
    void update();
    void renderGUI();
    void renderWorld();
    void render();
};