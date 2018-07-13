#include <array>
#include <cassert>
#include <iostream>
#include <optional>

#include <SFML/Graphics.hpp>

#include "AppData.h"
#include "Geometry.h"

#define FPS_LIMIT 10

void handleEvents(sf::RenderWindow& window, AppData& data) {
    sf::Event event{};
    sf::Rect<float> triangulateButton(15, 80, 75, 25);
    bool& show = data.uiShow;
    Geometry& allGeometry = data.geometry;

    while(window.pollEvent(event)) {
        switch(event.type) {
        case sf::Event::Closed:
            window.close();
            break;
        case sf::Event::KeyPressed:
            if(event.key.code == sf::Keyboard::Escape) {
                window.close();
            } else if(event.key.code == sf::Keyboard::H) {
                show = !show;
            } else if(event.key.code == sf::Keyboard::D) {
                const auto& points = allGeometry.getPoints();
                for(const auto& p : points) {
                    std::cout << "sf::Vector2f(" << p.x << "," << p.y << "), ";
                }
            } else if(event.key.code == sf::Keyboard::R) {
                data.pattern.reset();
            } else if(event.key.code == sf::Keyboard::I) {
                data.pattern.getNewValues();
            }
            break;
        case sf::Event::MouseButtonPressed: {
            const auto xPos = float(event.mouseButton.x);
            const auto yPos = float(event.mouseButton.y);
            // Triangulate button
            if(triangulateButton.contains(xPos, yPos)) {
                std::cout << "Triangulate!"
                          << "\n";
                allGeometry.triangulate();
            } else if(event.mouseButton.button == sf::Mouse::Right) {
                allGeometry.insertPoint(sf::Vector2f(xPos, yPos));
            }
            break;
        }

        default:
            break;
        }
    }
}

int main() {
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    sf::RenderWindow window(sf::VideoMode(800, 600), std::string("AVG project Stepan Hojdar"), sf::Style::Default,
                            settings);
    window.setFramerateLimit(FPS_LIMIT);

    AppData data;

    while(window.isOpen()) {
        handleEvents(window, data);

        window.clear();

        data.draw(window);

        window.display();
    }

    return 0;
}
