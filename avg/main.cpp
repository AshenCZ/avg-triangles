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
    sf::RenderWindow window(sf::VideoMode(200, 200), std::string("AVG project Stepan Hojdar"));
    window.setSize(sf::Vector2u(640, 480));
    window.setFramerateLimit(FPS_LIMIT);

    sf::CircleShape shape(100.f);
    shape.setFillColor(sf::Color::Green);

    while(window.isOpen()) {
        handleEvents(window);

        window.clear();
        window.draw(shape);
        window.display();
    }

    return 0;
}
