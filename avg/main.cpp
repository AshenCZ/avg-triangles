#include <cassert>
#include <iostream>
#include <optional>
#include <array>

#include <SFML/Graphics.hpp>

#define FPS_LIMIT 10

struct Triangle {
    std::array<size_t, 3> vertexIndex;

    bool drawable = true;

    Triangle(size_t one, size_t two, size_t three) {
        vertexIndex[0] = one;
        vertexIndex[1] = two;
        vertexIndex[2] = three;
    };
};

class Geometry {
    std::vector<sf::Vector2f> mPoints;
    std::vector<Triangle> mTriangles;

    void fillGeometry() {
        mPoints = {sf::Vector2f(361, 179), sf::Vector2f(291, 251), sf::Vector2f(413, 264), sf::Vector2f(240, 158),
                   sf::Vector2f(172, 247), sf::Vector2f(332, 337), sf::Vector2f(483, 353), sf::Vector2f(411, 419),
                   sf::Vector2f(259, 381), sf::Vector2f(507, 160)};

        mTriangles.reserve(11);
        mTriangles.emplace_back(0, 1, 2);
        mTriangles.emplace_back(1, 3, 4);
        mTriangles.emplace_back(0, 1, 3);
        mTriangles.emplace_back(1, 4, 5);
        mTriangles.emplace_back(6, 5, 7);
        mTriangles.emplace_back(2, 1, 5);
        mTriangles.emplace_back(2, 5, 6);
        mTriangles.emplace_back(5, 7, 8);
        mTriangles.emplace_back(9, 2, 0);
        mTriangles.emplace_back(9, 2, 6);
        mTriangles.emplace_back(4, 5, 8);
    }

    void resetTriangleData()
    {
        // Reset triangles
        mTriangles.clear();
    }

    void triangulateAll() {
        if(mPoints.size() < 3) {
            return;
        }

        // Reset triangles
        resetTriangleData();

        // Build overarching triangle with flag !drawable
        // \todo

        // Insert point by point
        // for(auto& pt : mPoints) {
        //     triangulateOne(pt);
        // }
    }

    static float dot(const sf::Vector2f left, const sf::Vector2f right) {
        return left.x * right.x + left.y * right.y;
    }

    // Compute barycentric coordinates of 'point' in 'triangle' and check if it is inside. Return coordinates if inside, empty if outside.
    std::optional<std::array<float, 3>> checkBarycentricCoordinates(const Triangle& triangle, const sf::Vector2f& point, const float tolerance = 0.00001f) const {
        const sf::Vector2f& a = mPoints[triangle.vertexIndex[0]];
        const sf::Vector2f& b = mPoints[triangle.vertexIndex[1]];
        const sf::Vector2f& c = mPoints[triangle.vertexIndex[2]];

        const sf::Vector2f v0 = b - a;
        const sf::Vector2f v1 = c - a;
        const sf::Vector2f v2 = point - a;

        const float d00 = dot(v0, v0);
        const float d01 = dot(v0, v1);
        const float d11 = dot(v1, v1);
        const float d20 = dot(v2, v0);
        const float d21 = dot(v2, v1);
        const float denominator = d00 * d11 - d01 * d01;

        if (denominator == 0) {
            return {};
        }

        const float denominatorFraction = 1.f / denominator;
        const float v = (d11 * d20 - d01 * d21) * denominatorFraction;
        const float w = (d00 * d21 - d01 * d20) * denominatorFraction;

        // The point is inside the triangle IFF u + v + w == 1 && 0 <= u,v,w <= 1
        if (std::min(v, w) >= -tolerance && std::max(v, w) <= 1.f + tolerance && v + w <= 1.f + tolerance) {
            return std::array<float, 3>({ 1.0f - v - w, v, w });
        } else {
            return {};
        }
    }

    // Returns the index of the triangle 'point' lies in
    size_t findTriangle(const sf::Vector2f point) const
    {
        for(size_t i = 0; i < mTriangles.size(); ++i)
        {
            const auto& triangle = mTriangles[i];
            const auto coordsMaybe = checkBarycentricCoordinates(triangle, point);
            if(coordsMaybe.has_value()) {
                return i;
            }
        }
        return 0;
    }

    void triangulateOne(const sf::Vector2f point)
    {
        const size_t insideTriangleInd = findTriangle(point);
        const size_t priorTriSize = mTriangles.size();
        assert(insideTriangleInd < mTriangles.size());

        const std::array<size_t, 3> vertexIndices =mTriangles[insideTriangleInd].vertexIndex;
        mTriangles.erase(mTriangles.begin() + insideTriangleInd);
        assert(mPoints.back().x == point.x);
        assert(mPoints.back().y == point.y);

        const size_t pointIndex = mPoints.size() - 1;
        mTriangles.emplace_back(vertexIndices[0],vertexIndices[1],pointIndex);
        mTriangles.emplace_back(pointIndex,vertexIndices[1],vertexIndices[2]);
        mTriangles.emplace_back(vertexIndices[0],pointIndex,vertexIndices[2]);

        assert(priorTriSize + 2 == mTriangles.size());
    }

   public:
    Geometry() {
        fillGeometry();
    }

    void insertNewPoint(sf::Vector2f point) {
        mPoints.emplace_back(point.x, point.y);

        // Step of incremental triangulation
        triangulateOne(point);
    }

    const std::vector<sf::Vector2f>& getPoints() const {
        return mPoints;
    }

    const std::vector<Triangle>& getTriangles() const {
        return mTriangles;
    }

    void triangulate() {
        triangulateAll();
    }
};

void handleEvents(sf::RenderWindow& window, Geometry& allGeometry) {
    sf::Event event{};
    sf::Rect<float> triangulateButton(15, 80, 75, 25);
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
        case sf::Event::MouseButtonPressed: {
            const auto xPos = float(event.mouseButton.x);
            const auto yPos = float(event.mouseButton.y);
            // Triangulate button
            if(triangulateButton.contains(xPos, yPos)) {
                std::cout << "Triangulate!"
                          << "\n";
                allGeometry.triangulate();
            } else if(event.mouseButton.button == sf::Mouse::Right) {
                allGeometry.insertNewPoint(sf::Vector2f(xPos, yPos));
            }
            break;
        }

        default:
            break;
        }
    }
}

void fillUi(std::vector<sf::Text>& uiText, std::vector<sf::RectangleShape>& uiRects, const sf::Font& font) {
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

int main() {
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    sf::RenderWindow window(sf::VideoMode(800, 600), std::string("AVG project Stepan Hojdar"), sf::Style::Default,
                            settings);
    window.setFramerateLimit(FPS_LIMIT);

    sf::Font font;
    if(!font.loadFromFile("../fonts/OpenSans-Regular.ttf")) {
    }

    sf::CircleShape shape(3.f);
    shape.setFillColor(sf::Color::Green);
    shape.setOrigin(shape.getRadius(), shape.getRadius());

    Geometry allGeometry;

    std::vector<sf::Text> uiText;
    std::vector<sf::RectangleShape> uiRects;
    fillUi(uiText, uiRects, font);

    const auto& points = allGeometry.getPoints();
    const auto& triangles = allGeometry.getTriangles();

    while(window.isOpen()) {
        handleEvents(window, allGeometry);

        window.clear();

        // Draw the points
        for(const auto& pt : allGeometry.getPoints()) {
            shape.setPosition(pt.x, pt.y);
            window.draw(shape);
        }

        // Draw the triangle sides
        for(const Triangle& tri : allGeometry.getTriangles()) {
            if(!tri.drawable) {
                continue;
            }

            sf::Vertex line1[] = {sf::Vertex(points[tri.vertexIndex[0]]), sf::Vertex(points[tri.vertexIndex[1]])};
            sf::Vertex line2[] = {sf::Vertex(points[tri.vertexIndex[1]]), sf::Vertex(points[tri.vertexIndex[2]])};
            sf::Vertex line3[] = {sf::Vertex(points[tri.vertexIndex[2]]), sf::Vertex(points[tri.vertexIndex[0]])};

            window.draw(line1, 2, sf::Lines);
            window.draw(line2, 2, sf::Lines);
            window.draw(line3, 2, sf::Lines);
        }

        // Draw UI
        uiText[0].setString("Triangles: " + std::to_string(triangles.size()) +
                            "\nPoints: " + std::to_string(points.size()));
        for(const auto& rect : uiRects) {
            window.draw(rect);
        }
        for(const auto& text : uiText) {
            window.draw(text);
        }

        window.display();
    }

    return 0;
}
