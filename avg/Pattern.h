#pragma once

#include <vector>

#include <SFML/Graphics.hpp>

using Edge = std::pair<size_t, size_t>;

class Pattern {
    std::vector<Edge> mEdgesInLetter = {Edge(0, 1), Edge(1, 2), Edge(2, 3), Edge(3, 4),
                                        Edge(4, 5), Edge(5, 6), Edge(6, 7), Edge(7, 0)};

    std::vector<sf::Vector2f> mPointsInLetter = {sf::Vector2f(70, 450),  sf::Vector2f(140, 450), sf::Vector2f(140, 470),
                                                 sf::Vector2f(115, 470), sf::Vector2f(115, 530), sf::Vector2f(95, 535),
                                                 sf::Vector2f(95, 470),  sf::Vector2f(70, 470)};

    size_t mCount = 1;
    sf::Vector2f mDirection;

   public:
    explicit Pattern(std::vector<Edge>& e, const size_t c, const sf::Vector2f v)
        : mEdgesInLetter(e), mCount(c), mDirection(v){};
    Pattern() = default;

    const std::vector<Edge>& getEdges() const {
        return mEdgesInLetter;
    }
    const std::vector<sf::Vector2f>& getPoints() const {
        return mPointsInLetter;
    }

    void draw(sf::RenderWindow& window)
    {
        // Draw the pattern
        for(const auto& e : mEdgesInLetter) {
            sf::Vertex line[] = {sf::Vertex(mPointsInLetter[e.first], sf::Color::Red),
                                 sf::Vertex(mPointsInLetter[e.second], sf::Color::Red)};
            window.draw(line, 2, sf::Lines);
        }
    }
};