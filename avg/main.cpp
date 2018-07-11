#include <array>
#include <cassert>
#include <iostream>
#include <optional>

#include <SFML/Graphics.hpp>

#define FPS_LIMIT 10

using Edge = std::pair<size_t, size_t>;

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

    static bool circumCircleContains(const sf::Vector2f& v, const sf::Vector2f& p1, const sf::Vector2f& p2,
                                     const sf::Vector2f& p3) {
        const float ab = p1.x * p1.x + p1.y * p1.y;
        const float cd = p2.x * p2.x + p2.y * p2.y;
        const float ef = p3.x * p3.x + p3.y * p3.y;

        const float circum_x = (ab * (p3.y - p2.y) + cd * (p1.y - p3.y) + ef * (p2.y - p1.y)) /
                               (p1.x * (p3.y - p2.y) + p2.x * (p1.y - p3.y) + p3.x * (p2.y - p1.y));
        const float circum_y = (ab * (p3.x - p2.x) + cd * (p1.x - p3.x) + ef * (p2.x - p1.x)) /
                               (p1.y * (p3.x - p2.x) + p2.y * (p1.x - p3.x) + p3.y * (p2.x - p1.x));

        const auto distance = [](const sf::Vector2f p, const sf::Vector2f v) -> float {
            float dx = p.x - v.x;
            float dy = p.y - v.y;
            return dx * dx + dy * dy;
        };

        const sf::Vector2f circum(0.5f * (circum_x), 0.5f * (circum_y));
        const float circum_radius = distance(p1, circum);
        const float dist = distance(v, circum);
        return dist <= circum_radius;
    }

    void fillGeometry() {
        // mPoints = {sf::Vector2f(361, 179), sf::Vector2f(291, 251), sf::Vector2f(413, 264), sf::Vector2f(240, 158),
        //           sf::Vector2f(172, 247), sf::Vector2f(332, 337), sf::Vector2f(483, 353), sf::Vector2f(411, 419),
        //           sf::Vector2f(259, 381), sf::Vector2f(507, 160)};
        // mPoints = {sf::Vector2f(361, 179), sf::Vector2f(291, 251), sf::Vector2f(413, 264)};

        mPoints = {};
        mPoints.emplace_back(sf::Vector2f(400, -1000));
        mPoints.emplace_back(sf::Vector2f(-400, 700));
        mPoints.emplace_back(sf::Vector2f(1200, 700));
        const size_t bigTop = mPoints.size() - 3;
        const size_t bigLeft = mPoints.size() - 2;
        const size_t bigRight = mPoints.size() - 1;
        assert(mPoints[bigTop].x == 400 && mPoints[bigTop].y == -1000);
        assert(mPoints[bigLeft].x == -400 && mPoints[bigLeft].y == 700);
        assert(mPoints[bigRight].x == 1200 && mPoints[bigLeft].y == 700);
        mTriangles.emplace_back(bigTop, bigLeft, bigRight);
        mTriangles[0].drawable = false;

        // mTriangles.emplace_back(0, 1, 2);

        // check for coutnerclockwise!
        /// mTriangles.reserve(11);
        /// mTriangles.emplace_back(1, 3, 4);
        /// mTriangles.emplace_back(0, 1, 3);
        /// mTriangles.emplace_back(1, 4, 5);
        /// mTriangles.emplace_back(6, 5, 7);
        /// mTriangles.emplace_back(2, 1, 5);
        /// mTriangles.emplace_back(2, 5, 6);
        /// mTriangles.emplace_back(5, 7, 8);
        /// mTriangles.emplace_back(9, 2, 0);
        /// mTriangles.emplace_back(9, 2, 6);
        /// mTriangles.emplace_back(4, 5, 8);
    }

    void resetTriangleData() {
        // Reset triangles
        mTriangles.clear();
    }

    size_t findNeighbourTriangle(const size_t triangleIndex, const size_t vertexIndex1,
                                 const size_t vertexIndex2) const {
        for(size_t i = 0; i < mTriangles.size(); ++i) {
            if(i == triangleIndex) {
                continue;
            }

            const Triangle& tested = mTriangles[i];
            bool v1Correct = false;
            bool v2Correct = false;
            for(int v = 0; v < 3; ++v) {
                if(tested.vertexIndex[v] == vertexIndex1) {
                    v1Correct = true;
                }
                if(tested.vertexIndex[v] == vertexIndex2) {
                    v2Correct = true;
                }
            }
            if(v1Correct && v2Correct) {
                return i;
            }
        }
        return std::numeric_limits<size_t>::max();
    }

    struct EdgeSearchResult {
        size_t triangleIndex = std::numeric_limits<size_t>::max();
        size_t vertexIndex1 = std::numeric_limits<size_t>::max();
        size_t vertexIndex2 = std::numeric_limits<size_t>::max();

        EdgeSearchResult() = default;
        EdgeSearchResult(size_t a, size_t b, size_t c) : triangleIndex(a), vertexIndex1(b), vertexIndex2(c){};
    };

    std::vector<Edge> flip(size_t triangleIndex, const size_t vert1, const size_t vert2) {
        const Triangle& triangle = mTriangles[triangleIndex];
        const size_t vertexIndex1 = triangle.vertexIndex[vert1];
        const size_t vertexIndex2 = triangle.vertexIndex[vert2];
        const size_t aIndex = triangle.vertexIndex[3 - vert1 - vert2];  // 0 1 2, I want the third, sum = 3

        // Find second triangle with triangle[vert1] and triangle[vert2]
        const size_t neighbourIndex = findNeighbourTriangle(triangleIndex, vertexIndex1, vertexIndex2);
        if(neighbourIndex == std::numeric_limits<size_t>::max()) {
            return std::vector<Edge>();
        }
        assert(neighbourIndex < mTriangles.size());
        const Triangle& neighbour = mTriangles[neighbourIndex];

        // Check condition
        sf::Vector2f d;
        size_t dIndex = std::numeric_limits<size_t>::max();
        if(neighbour.vertexIndex[0] != triangle.vertexIndex[0] && neighbour.vertexIndex[0] != triangle.vertexIndex[1] &&
           neighbour.vertexIndex[0] != triangle.vertexIndex[2]) {
            d = mPoints[neighbour.vertexIndex[0]];
            dIndex = neighbour.vertexIndex[0];
        } else if(neighbour.vertexIndex[1] != triangle.vertexIndex[0] &&
                  neighbour.vertexIndex[1] != triangle.vertexIndex[1] &&
                  neighbour.vertexIndex[1] != triangle.vertexIndex[2]) {
            d = mPoints[neighbour.vertexIndex[1]];
            dIndex = neighbour.vertexIndex[1];
        } else if(neighbour.vertexIndex[2] != triangle.vertexIndex[0] &&
                  neighbour.vertexIndex[2] != triangle.vertexIndex[1] &&
                  neighbour.vertexIndex[2] != triangle.vertexIndex[2]) {
            d = mPoints[neighbour.vertexIndex[2]];
            dIndex = neighbour.vertexIndex[2];
        } else {
            assert(false);
        }
        assert(dIndex != std::numeric_limits<size_t>::max());

        const bool flipEdge = circumCircleContains(d, mPoints[triangle.vertexIndex[0]],
                                                   mPoints[triangle.vertexIndex[1]], mPoints[triangle.vertexIndex[2]]);
        std::cout << "Flip: " << flipEdge << "\n";
        std::cout << "Flip has 4 vertices: " << vertexIndex1 << " " << vertexIndex2 << " " << aIndex << " " << dIndex
                  << "\n";
        // If false, return
        if(!flipEdge) {
            return std::vector<Edge>();
        }
        // If true
        else {
            // Flip
            if(triangleIndex > neighbourIndex)
            {
                mTriangles.erase(mTriangles.begin() + triangleIndex);
                mTriangles.erase(mTriangles.begin() + neighbourIndex);                
            } else
            {
                mTriangles.erase(mTriangles.begin() + neighbourIndex);                
                mTriangles.erase(mTriangles.begin() + triangleIndex);
            }

            mTriangles.emplace_back(aIndex, vertexIndex1, dIndex);
            mTriangles.emplace_back(aIndex, dIndex, vertexIndex2);
            // And test the neighbouring four! triangles too

            std::vector<Edge> returnVal;
            returnVal.push_back({aIndex, vertexIndex1});
            returnVal.push_back({vertexIndex1, dIndex});

            // flip(mTriangles.size() - 2, 0, 1);
            // flip(mTriangles.size() - 2, 1, 2);

            // flip(mTriangles.size() - 1, 1, 2);
            // flip(mTriangles.size() - 1, 2, 0);
            returnVal.push_back({dIndex, vertexIndex2});
            returnVal.push_back({vertexIndex2, aIndex});
            return returnVal;
        }
    }

    void triangulateAll() {
        if(mPoints.size() < 3) {
            return;
        }

        // Reset triangles
        mTriangles.clear();

        // Move points
        auto oldPoints = std::move(mPoints);
        mPoints.clear();
        mPoints.reserve(oldPoints.size());

        // Insert point by point
        for(auto& pt : oldPoints) {
            mPoints.emplace_back(pt.x, pt.y);
            const auto newTrianglesMaybe = insertPointIntoTriangle(pt);
            if(!newTrianglesMaybe.has_value()) {
                continue;
            }
            std::array<Edge, 3> newTriangles = newTrianglesMaybe.value();
            queueFlip(newTriangles);

            //flip(newTriangles[0], 0, 1);
            //flip(newTriangles[1], 1, 2);
            //flip(newTriangles[2], 0, 2);
        }
    }

    static float dot(const sf::Vector2f left, const sf::Vector2f right) {
        return left.x * right.x + left.y * right.y;
    }

    // Compute barycentric coordinates of 'point' in 'triangle' and check if it is inside. Return coordinates if inside,
    // empty if outside. Code based on the following link. Edited to not only compute but also check the viability.
    // https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates,
    std::optional<std::array<float, 3>> checkBarycentricCoordinates(const Triangle& triangle, const sf::Vector2f& point,
                                                                    const float tolerance = 0.0000001f) const {
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

        if(denominator == 0) {
            return {};
        }

        const float denominatorFraction = 1.f / denominator;
        const float v = (d11 * d20 - d01 * d21) * denominatorFraction;
        const float w = (d00 * d21 - d01 * d20) * denominatorFraction;

        // The point is inside the triangle IFF u + v + w == 1 && 0 <= u,v,w <= 1
        if(std::min(v, w) >= -tolerance && std::max(v, w) <= 1.f + tolerance && v + w <= 1.f + tolerance) {
            return std::array<float, 3>({1.0f - v - w, v, w});
        } else {
            return {};
        }
    }

    // Returns the index of the triangle 'point' lies in
    std::optional<size_t> findTriangle(const sf::Vector2f point) const {
        for(size_t i = 0; i < mTriangles.size(); ++i) {
            const auto& triangle = mTriangles[i];
            const auto coordsMaybe = checkBarycentricCoordinates(triangle, point);
            if(coordsMaybe.has_value()) {
                return i;
            }
        }
        return {};
    }

    std::optional<std::array<Edge, 3>> insertPointIntoTriangle(const sf::Vector2f point) {
        if(mPoints.size() < 3) {
            return {};
        } else if(mPoints.size() == 3) {
            mTriangles.emplace_back(0, 1, 2);
            return {};
        }
        const auto indexMaybe = findTriangle(point);
        size_t insideTriangleInd = 0;
        if(!indexMaybe.has_value()) {
            return {};
        } else {
            insideTriangleInd = indexMaybe.value();
        }
        const size_t priorTriSize = mTriangles.size();
        assert(insideTriangleInd < mTriangles.size());

        const std::array<size_t, 3> vertexIndices = mTriangles[insideTriangleInd].vertexIndex;
        mTriangles.erase(mTriangles.begin() + insideTriangleInd);
        assert(mPoints.back().x == point.x);
        assert(mPoints.back().y == point.y);

        const size_t pointIndex = mPoints.size() - 1;
        mTriangles.emplace_back(vertexIndices[0], vertexIndices[1], pointIndex);
        mTriangles.emplace_back(pointIndex, vertexIndices[1], vertexIndices[2]);
        mTriangles.emplace_back(vertexIndices[0], pointIndex, vertexIndices[2]);

        assert(priorTriSize + 2 == mTriangles.size());
        return std::array<Edge, 3>({Edge({vertexIndices[0], vertexIndices[1]}),
                                    Edge({vertexIndices[1], vertexIndices[2]}),
                                    Edge({vertexIndices[0], vertexIndices[2]})});
    }

    EdgeSearchResult findTriangleWithEdge(const Edge& edge) const {
        const size_t vert1 = edge.first;
        const size_t vert2 = edge.second;

        EdgeSearchResult result;

        for(size_t i = 0; i < mTriangles.size(); ++i) {
            const Triangle& tri = mTriangles[i];
            bool found1 = false;
            bool found2 = false;

            if(tri.vertexIndex[0] == vert1 || tri.vertexIndex[1] == vert1 || tri.vertexIndex[2] == vert1) {
                found1 = true;
            }
            if(tri.vertexIndex[0] == vert2 || tri.vertexIndex[1] == vert2 || tri.vertexIndex[2] == vert2) {
                found2 = true;
            }
            if(found1 && found2) {
                result.triangleIndex = i;
                if(tri.vertexIndex[0] == vert1) {
                    result.vertexIndex1 = 0;
                } else if(tri.vertexIndex[1] == vert1) {
                    result.vertexIndex1 = 1;
                } else if(tri.vertexIndex[2] == vert1) {
                    result.vertexIndex1 = 2;
                } else {
                    assert(false);
                }

                if(tri.vertexIndex[0] == vert2) {
                    result.vertexIndex2 = 0;
                } else if(tri.vertexIndex[1] == vert2) {
                    result.vertexIndex2 = 1;
                } else if(tri.vertexIndex[2] == vert2) {
                    result.vertexIndex2 = 2;
                } else {
                    assert(false);
                }
                return result;
            }
        }
        assert(false);
        return result;
    }

    void queueFlip(const std::array<Edge, 3>& edgesToCheck) {
        std::vector<Edge> queue;
        std::copy(edgesToCheck.begin(), edgesToCheck.end(), std::back_inserter(queue));
        assert(queue.size() == 3);

        while(!queue.empty()) {
            // check one edge
            const Edge current = queue.back();
            const EdgeSearchResult result = findTriangleWithEdge(current);
            std::vector<Edge> newEdgesToFlip = flip(result.triangleIndex, result.vertexIndex1, result.vertexIndex2);

            // remove from list
            queue.pop_back();

            // add new flipped
            for(Edge& e : newEdgesToFlip)
            {
                queue.push_back(e);
            }
        }
    }

   public:
    Geometry() {
        fillGeometry();
    }

    void insertNewPoint(sf::Vector2f point) {
        mPoints.emplace_back(point.x, point.y);

        // Step of incremental triangulation
        const auto newTrianglesMaybe = insertPointIntoTriangle(point);
        if(!newTrianglesMaybe.has_value()) {
            return;
        }
        std::array<Edge, 3> newTriangles = newTrianglesMaybe.value();
        queueFlip(newTriangles);


        //flip(newTriangles[1], 1, 2);
        //flip(newTriangles[2], 0, 2);
    }

    const std::vector<sf::Vector2f>& getPoints() const {
        return mPoints;
    }

    std::vector<Triangle>& getTriangles() {
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
        for(Triangle& tri : allGeometry.getTriangles()) {
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
