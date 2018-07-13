#pragma once

#include <vector>

#include <SFML/Graphics.hpp>

using Edge = std::pair<size_t, size_t>;

class Pattern {
    std::vector<Edge> mEdgesInLetter = {Edge(0, 1), Edge(1, 2), Edge(2, 3), Edge(3, 4),
                                        Edge(4, 5), Edge(5, 6), Edge(6, 7), Edge(7, 0)};

    std::vector<sf::Vector2f> mPointsInLetter = {sf::Vector2f(0, 0),  sf::Vector2f(70, 0), sf::Vector2f(70, 20),
                                                 sf::Vector2f(45, 20), sf::Vector2f(45, 80), sf::Vector2f(25, 85),
                                                 sf::Vector2f(25, 20),  sf::Vector2f(0, 20)};

    size_t mCount = 4;
    sf::Vector2f mDirection = sf::Vector2f(75, 75);
    sf::Vector2f mStart = sf::Vector2f(150,150);

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
        for(size_t i = 0; i < mCount; ++i)
        {
            const sf::Vector2f currentShift = mStart + sf::Vector2f(i * mDirection.x, i * mDirection.y);

            // Draw the pattern
            for(const auto& e : mEdgesInLetter) {
                sf::Vertex line[] = {sf::Vertex(mPointsInLetter[e.first] + currentShift, sf::Color::Red),
                                     sf::Vertex(mPointsInLetter[e.second] + currentShift, sf::Color::Red)};
                window.draw(line, 2, sf::Lines);
            }
        }        
    }

    void reset()
    {
        mCount = 1;
        mDirection = sf::Vector2f(75, 75);
        mStart = sf::Vector2f(150, 150);
    }

    void getNewValues()
    {
        std::cout << "New number of pattern iterations (max 10): ";
        std::cin >> mCount;
        assert(mCount < 10);

        std::cout << "New starting position (x,y), max (800,600): ";
        float x, y;
        std::cin >> x >> y;
        mStart = sf::Vector2f(x, y);

        std::cout << "New direction (x,y), max (800,600): ";
        std::cin >> x >> y;
        mDirection = sf::Vector2f(x, y);
        std::cout << "Pattern set.\n";
    }
};