#include <cassert>
#include <iostream>

#include <SFML/Graphics.hpp>

#define FPS_LIMIT 10

void handleEvents(sf::RenderWindow& window) {
    sf::Event event{};
    while(window.pollEvent(event)) {
        switch(event.type) {
        case sf::Event::Closed:
            window.close();
            break;
        case sf::Event::KeyPressed:
            if(event.key.code == sf::Keyboard::Escape) {
                window.close();
            }
            break;
        case sf::Event::MouseButtonPressed:
            std::cout << "click (" << event.mouseButton.x << ", " << event.mouseButton.y << ")\n";
            break;
        default:
            break;
        }
    }
}

int main() {
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    sf::RenderWindow window(sf::VideoMode(800, 600), std::string("AVG project Stepan Hojdar"), sf::Style::Default, settings);
    window.setFramerateLimit(FPS_LIMIT);

    sf::Font font;
    if(!font.loadFromFile("../fonts/OpenSans-Regular.ttf")) {}

    sf::CircleShape shape(100.f);
    shape.setFillColor(sf::Color::Green);

    while(window.isOpen()) {
        handleEvents(window);

        window.clear();
        shape.setPosition(50,50);
        window.draw(shape);

        sf::Text text;
        text.setFont(font);
        text.setCharacterSize(16);
        text.setString("Triangles: 0");
        text.setFillColor(sf::Color::White);

        window.draw(text);
        window.display();
    }

    return 0;
}
