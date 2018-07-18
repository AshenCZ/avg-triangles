#pragma once

#include <array>
#include <cassert>
#include <optional>
#include <vector>

#include <SFML/Graphics.hpp>

#include "Geometry.h"
#include "Pattern.h"

class AppData {
   private:
    sf::Font font;
    std::vector<sf::Text> uiText;
    std::vector<sf::RectangleShape> uiRects;
    sf::CircleShape shape;

   public:
    Geometry geometry;
    Pattern pattern;
    bool uiShow = false;

    AppData() {
        if(!font.loadFromFile("../fonts/OpenSans-Regular.ttf")) {
        }
        fillUi(uiText, uiRects, font);


        shape.setRadius(3.f);
        shape.setFillColor(sf::Color::Green);
        shape.setOrigin(shape.getRadius(), shape.getRadius());
    }

    void draw(sf::RenderWindow& window) {
        drawUi(window);
        drawGeometry(window);
        if(uiShow) {
            pattern.draw(window);
        }
    }

    void insertDummyData() {
        // Fill Geometry
        auto pointsToInsert = {sf::Vector2f(361, 179), sf::Vector2f(291, 251), sf::Vector2f(413, 264),
                               sf::Vector2f(240, 158), sf::Vector2f(172, 247), sf::Vector2f(332, 337),
                               sf::Vector2f(483, 353), sf::Vector2f(411, 419), sf::Vector2f(259, 381),
                               sf::Vector2f(507, 160)};
        // auto pointsToInsert = {sf::Vector2f(298, 242), sf::Vector2f(161, 353), sf::Vector2f(362, 394)};

        for(const auto& pt : pointsToInsert) {
            geometry.insertPoint(pt);
        }
    }

    void insertPattern() {
        const size_t patternCount = pattern.getCount();
        assert(patternCount > 0);

        // Gather all points
        std::vector<sf::Vector2f> patternPoints;
        for(size_t i = 0; i < patternCount; ++i) {
            pattern.getPointsNumber(i, patternPoints);
        }

        const size_t sizeBeforeInsert = geometry.getPoints().size();
        const size_t patternSize = pattern.getPoints().size();
        assert(patternPoints.size() == patternSize * patternCount);

        // Insert all points
        for(const auto& point : patternPoints) {
            geometry.insertPoint(point, true);
        }

        // Cut all edges
        for(size_t i = 0; i < patternCount; ++i)
        {
            const size_t patternEdgeOffset = i * patternSize;
            const size_t offset = sizeBeforeInsert + patternEdgeOffset;
            for(const auto& edge : pattern.getEdges()) {
                Edge e(edge.first + offset, edge.second + offset);
                geometry.inputEdge(e);
            }
        }
    }

   private:
    static void fillUi(std::vector<sf::Text>& uiText, std::vector<sf::RectangleShape>& uiRects, const sf::Font& font) {
        sf::RectangleShape triangulateButton(sf::Vector2f(75, 25));
        triangulateButton.setPosition(15, 80);
        triangulateButton.setFillColor(sf::Color(50, 50, 50));
        triangulateButton.setOutlineColor(sf::Color::White);
        triangulateButton.setOutlineThickness(1);
        uiRects.push_back(triangulateButton);

        sf::Text text;
        text.setFont(font);
        text.setPosition(15, 15);
        text.setCharacterSize(16);
        text.setFillColor(sf::Color::White);
        uiText.push_back(text);

        sf::Text triangulateButtonText;
        triangulateButtonText.setFont(font);
        triangulateButtonText.setPosition(20, 83);
        triangulateButtonText.setCharacterSize(12);
        triangulateButtonText.setString("Triangulate");
        triangulateButtonText.setFillColor(sf::Color::White);
        uiText.push_back(triangulateButtonText);
    }

    void drawUi(sf::RenderWindow& window) {
        // Draw UI
        uiText[0].setString("Triangles: " + std::to_string(geometry.getTriangles().size()) +
                            "\nPoints: " + std::to_string(geometry.getPoints().size()) +
                            "\nPress H to hide/show encapsulating triangle");
        for(const auto& rect : uiRects) {
            window.draw(rect);
        }
        for(const auto& text : uiText) {
            window.draw(text);
        }
    }

    void drawGeometry(sf::RenderWindow& window) {
        // Draw the triangle sides
        auto& points = geometry.getPoints();
        for(Triangle& tri : geometry.getTriangles()) {
            if(!uiShow) {  // Show or hide hidden edges
                if(!tri.drawable) {
                    continue;
                }

                bool valid = true;
                if(tri.vertexIndex[0] == 0 || tri.vertexIndex[0] == 1 || tri.vertexIndex[0] == 2) {
                    valid = false;
                }
                if(tri.vertexIndex[1] == 0 || tri.vertexIndex[1] == 1 || tri.vertexIndex[1] == 2) {
                    valid = false;
                }
                if(tri.vertexIndex[2] == 0 || tri.vertexIndex[2] == 1 || tri.vertexIndex[2] == 2) {
                    valid = false;
                }
                if(!valid) {
                    tri.drawable = false;
                    continue;
                }
            }
            std::array<std::pair<size_t, size_t>, 3> pairs = {std::pair<size_t, size_t>(0, 1), {1, 2}, {2, 0}};
            for(const auto& side : pairs) {
                sf::Vertex line[] = {sf::Vertex(points[tri.vertexIndex[side.first]], sf::Color::White),
                                     sf::Vertex(points[tri.vertexIndex[side.second]], sf::Color::White)};
                window.draw(line, 2, sf::Lines);
            }
        }

        // Draw the points
        for(const auto& pt : geometry.getPoints()) {
            shape.setPosition(pt.x, pt.y);
            window.draw(shape);
        }
    }
};
