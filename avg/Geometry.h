#pragma once

#include <array>
#include <cassert>
#include <deque>
#include <iostream>
#include <optional>
#include <unordered_map>
#include <vector>

#include <SFML/Graphics.hpp>

using Edge = std::pair<size_t, size_t>;

struct Triangle {
    std::array<size_t, 3> vertexIndex;

    bool drawable = true;

    explicit Triangle(size_t one, size_t two, size_t three) : vertexIndex({one, two, three}), drawable(true){};
};

struct pairhash {
   public:
    template <typename T, typename U>
    std::size_t operator()(const std::pair<T, U>& x) const {
        return std::hash<T>()(x.first) ^ std::hash<U>()(x.second);
    }
};

class ImportantEdges {
    /// Pointers to Geometry's mPoints
    std::vector<Edge> importantEdges;

    bool contains(const Edge e) const {
        for(const Edge& edge : importantEdges) {
            if(edge.first == e.first && edge.second == e.second) {
                return true;
            }
            if(edge.second == e.first && edge.first == e.second) {
                return true;
            }
        }
        return false;
    }

   public:
    ImportantEdges() = default;

    void insertEdge(const Edge e) {
        // Check for duplicates
        if(contains(e)) {
            return;
        }

        // Insert new important edge
        importantEdges.emplace_back(e);
    }

    bool isImportant(const Edge edge) const {
        return contains(edge);
    }

    void reset() {
        importantEdges.clear();
    }

    void removeEdge(const Edge edge) {
        for(size_t i = 0; i < importantEdges.size(); ++i) {
            const Edge& current = importantEdges[i];
            if(current.first == edge.first && current.second == edge.second) {
                importantEdges.erase(importantEdges.begin() + i);
            }
        }
    }
};

class Geometry {
    std::vector<sf::Vector2f> mPoints;
    std::vector<Triangle> mTriangles;


   public:
    ImportantEdges mImportant;

    Geometry() {
        fillGeometry();
    }

    void insertPoint(const sf::Vector2f point, bool reTriangulate = true) {
        insertOnePoint(point, reTriangulate);
    }

    const std::vector<sf::Vector2f>& getPoints() const {
        return mPoints;
    }

    std::vector<Triangle>& getTriangles() {
        return mTriangles;
    }

    void triangulate() {
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
            insertOnePoint(pt);
        }
    }

    void reset() {
        mPoints.clear();
        mTriangles.clear();
        fillGeometry();
        mImportant.reset();
    }

    void inputEdge(const Edge& edge) {
        insertEdge(edge);
    }

    /// Triangulates mPoints into new mTriangles, keeping all Edges in mImportant
    void constrainedTriangulation() {
        cdt();
    }

   private:
    /// Triangulation methods

    // Method taken from https://github.com/Bl4ckb0ne/delaunay-triangulation
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
        return dist < circum_radius;
    }

    void fillGeometry() {
        // Pre-fill the overlapping triangle
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
    }

    struct EdgeSearchResult {
        size_t triangleIndex = std::numeric_limits<size_t>::max();
        size_t vertexIndex1 = std::numeric_limits<size_t>::max();
        size_t vertexIndex2 = std::numeric_limits<size_t>::max();

        EdgeSearchResult() = default;
        EdgeSearchResult(size_t a, size_t b, size_t c) : triangleIndex(a), vertexIndex1(b), vertexIndex2(c){};
    };

    std::vector<EdgeSearchResult> findTriangleWithEdge(const Edge& edge) const {
        const size_t vert1 = edge.first;
        const size_t vert2 = edge.second;

        std::vector<EdgeSearchResult> resultList;

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
                EdgeSearchResult result;
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
                resultList.push_back(result);
            }
        }
        assert(resultList.size() <= 2 && !resultList.empty());
        return resultList;
    }

    std::vector<Edge> flip(size_t triangleIndex, const size_t vert1, const size_t vert2) {
        const Triangle triangle = mTriangles[triangleIndex];
        const size_t vertexIndex1 = triangle.vertexIndex[vert1];
        const size_t vertexIndex2 = triangle.vertexIndex[vert2];
        const size_t aIndex = triangle.vertexIndex[3 - vert1 - vert2];  // 0 1 2, I want the third, sum = 3

        // Find second triangle with triangle[vert1] and triangle[vert2]
        std::vector<EdgeSearchResult> incidents = findTriangleWithEdge(Edge(vertexIndex1, vertexIndex2));

        // If there is only one, no flipping required
        if(incidents.size() == 1) {
            assert(incidents[0].triangleIndex == triangleIndex);
            return std::vector<Edge>();
        }

        // Find which of the 2 found is new
        size_t whichTriIsNew = std::numeric_limits<size_t>::max();
        if(incidents[0].triangleIndex == triangleIndex) {
            whichTriIsNew = 1;
        } else if(incidents[1].triangleIndex == triangleIndex) {
            whichTriIsNew = 0;
        } else {
            assert(false);
        }
        assert(whichTriIsNew != std::numeric_limits<size_t>::max());

        const size_t neighbourIndex = incidents[whichTriIsNew].triangleIndex;  // index into mTriangles
        assert(neighbourIndex < mTriangles.size());
        const Triangle neighbour = mTriangles[neighbourIndex];

        // Check condition
        const size_t dIndex =
            neighbour.vertexIndex[3 - incidents[whichTriIsNew].vertexIndex1 - incidents[whichTriIsNew].vertexIndex2];
        assert(dIndex < mPoints.size());
        const bool flipEdge = circumCircleContains(mPoints[dIndex], mPoints[triangle.vertexIndex[0]],
                                                   mPoints[triangle.vertexIndex[1]], mPoints[triangle.vertexIndex[2]]);

        // If false, return
        if(!flipEdge) {
            return std::vector<Edge>();
        }
        // If true
        else {
            assert(aIndex != vertexIndex1 && vertexIndex1 != dIndex && aIndex != dIndex);
            assert(aIndex != dIndex && dIndex != vertexIndex2 && vertexIndex2 != aIndex);
            // Flip
            if(triangleIndex > neighbourIndex) {
                mTriangles.erase(mTriangles.begin() + triangleIndex);
                mTriangles.erase(mTriangles.begin() + neighbourIndex);
            } else {
                mTriangles.erase(mTriangles.begin() + neighbourIndex);
                mTriangles.erase(mTriangles.begin() + triangleIndex);
            }

            mTriangles.emplace_back(aIndex, vertexIndex1, dIndex);
            mTriangles.emplace_back(aIndex, dIndex, vertexIndex2);

            // And test the neighbouring four! triangles too
            std::vector<Edge> returnVal;
            returnVal.emplace_back(aIndex, vertexIndex1);
            returnVal.emplace_back(vertexIndex1, dIndex);
            returnVal.emplace_back(dIndex, vertexIndex2);
            returnVal.emplace_back(vertexIndex2, aIndex);
            return returnVal;
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
    std::optional<size_t> findTriangleWithPoint(const sf::Vector2f point) const {
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
        const auto indexMaybe = findTriangleWithPoint(point);
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
        assert(vertexIndices[0] != vertexIndices[1] && vertexIndices[1] != pointIndex &&
               pointIndex != vertexIndices[0]);

        mTriangles.emplace_back(pointIndex, vertexIndices[1], vertexIndices[2]);
        assert(pointIndex != vertexIndices[1] && vertexIndices[1] != vertexIndices[2] &&
               pointIndex != vertexIndices[2]);

        mTriangles.emplace_back(vertexIndices[0], pointIndex, vertexIndices[2]);
        assert(vertexIndices[0] != pointIndex && pointIndex != vertexIndices[2] &&
               vertexIndices[0] != vertexIndices[2]);

        assert(priorTriSize + 2 == mTriangles.size());
        return std::array<Edge, 3>({Edge({vertexIndices[0], vertexIndices[1]}),
                                    Edge({vertexIndices[1], vertexIndices[2]}),
                                    Edge({vertexIndices[0], vertexIndices[2]})});
    }

    void queueFlip(const std::array<Edge, 3>& edgesToCheck) {
        std::deque<Edge> queue;
        std::copy(edgesToCheck.begin(), edgesToCheck.end(), std::back_inserter(queue));
        assert(queue.size() == 3);

        while(!queue.empty()) {
            // check one edge
            const Edge current = queue.front();
            const EdgeSearchResult result = findTriangleWithEdge(current)[0];
            std::vector<Edge> newEdgesToFlip = flip(result.triangleIndex, result.vertexIndex1, result.vertexIndex2);

            // remove from list
            queue.pop_front();

            // add new flipped
            for(Edge& e : newEdgesToFlip) {
                queue.push_back(e);
            }
        }
    }

    void insertOnePoint(const sf::Vector2f point, bool retriangulate = true) {
        mPoints.emplace_back(point.x, point.y);

        // Step of incremental triangulation
        const auto newTrianglesMaybe = insertPointIntoTriangle(point);
        if(!newTrianglesMaybe.has_value()) {
            return;
        }
        if(retriangulate) {
            const std::array<Edge, 3> newTriangles = newTrianglesMaybe.value();
            queueFlip(newTriangles);
        }
    }

    /// Line - Intersection methods

    bool checkBelongLineSeg(const sf::Vector2f seg1, const sf::Vector2f seg2, const sf::Vector2f point, float& outCross,
                            const float EPS) const;

    std::vector<float> returnParametricIntersection(const Edge line1, const Edge line2);

    struct Intersection {
        /// Parameter of the edge to get the intersection point [0,1]
        float t = -1;

        /// Indicates whether a vertex of the triangle was intersected, contains which one {0, 1, 2}
        int vertexIntersected = -1;

        /// Indicates whether an edge of the triangle was intersected, contains which one {0, 1, 2}
        /// 0 = between 0 and 1
        /// 1 = between 1 and 2
        /// 2 = between 2 and 0
        int edgeIntersected = -1;

        Intersection() = default;

        Intersection(float tt, int vi, int ei) : t(tt), vertexIntersected(vi), edgeIntersected(ei){};
    };

    std::optional<std::vector<Intersection>> edgeIntersectsTriangle(const Edge& edge, const Triangle& triangle) {
        // Check for shared vertices
        size_t match1 = std::numeric_limits<size_t>::max();
        size_t match2 = std::numeric_limits<size_t>::max();
        for(size_t i = 0; i < 3; ++i) {
            if(edge.first == triangle.vertexIndex[i]) {
                match1 = i;
            }
            if(edge.second == triangle.vertexIndex[i]) {
                match2 = i;
            }
        }

        // 2 shared vertices? \todo ! mark edge as important
        if(match1 != std::numeric_limits<size_t>::max() && match2 != std::numeric_limits<size_t>::max()) {
            // completely hidden within and edge
            return {};
        }

        // shared vertex
        if(match1 != std::numeric_limits<size_t>::max() || match2 != std::numeric_limits<size_t>::max()) {
            // which is the same?
            size_t sharedTriangleIndex = 4;
            size_t outsideVertexPointsIndex = 4;
            float tt = 4;
            if(match1 != std::numeric_limits<size_t>::max()) {
                sharedTriangleIndex = match1;
                outsideVertexPointsIndex = edge.second;
                tt = 0;
            }
            if(match2 != std::numeric_limits<size_t>::max()) {
                sharedTriangleIndex = match2;
                outsideVertexPointsIndex = edge.first;
                tt = 1;
            }
            assert(sharedTriangleIndex != 4);
            assert(sharedTriangleIndex < 3);
            assert(outsideVertexPointsIndex != 4);

            const size_t sharedVertexPointsIndex = triangle.vertexIndex[sharedTriangleIndex];

            // sharedTriangleIndex 0 1 2 pointer to triangle.vertexIndex[]
            // outsideVertexPointsIndex pointer to mPoints
            // sharedVertexPointsIndex pointer to mPoints
            std::vector<float> ret;
            Intersection returnVal;
            if(sharedTriangleIndex == 0) {
                ret = returnParametricIntersection(edge, Edge(triangle.vertexIndex[1], triangle.vertexIndex[2]));
                returnVal.edgeIntersected = 1;
            } else if(sharedTriangleIndex == 1) {
                ret = returnParametricIntersection(edge, Edge(triangle.vertexIndex[2], triangle.vertexIndex[0]));
                returnVal.edgeIntersected = 2;
            } else if(sharedTriangleIndex == 2) {
                ret = returnParametricIntersection(edge, Edge(triangle.vertexIndex[0], triangle.vertexIndex[1]));
                returnVal.edgeIntersected = 0;
            } else {
                assert(false);
            }

            if(ret.empty()) {
                return {};
            }
            assert(ret.size() == 1);
            returnVal.t = ret[0];
            returnVal.vertexIntersected = -1;
            std::vector<Intersection> retVec;
            retVec.push_back(returnVal);
            assert(tt <= 1);
            retVec.emplace_back(tt, sharedTriangleIndex, -1);
            return retVec;
        }

        // Non-shared vertex
        std::vector<Intersection> returnValue;

        auto ret0 = returnParametricIntersection(edge, Edge(triangle.vertexIndex[0], triangle.vertexIndex[1]));
        if(!ret0.empty()) {
            returnValue.emplace_back(ret0[0], -1, 0);
        }
        auto ret1 = returnParametricIntersection(edge, Edge(triangle.vertexIndex[1], triangle.vertexIndex[2]));
        if(!ret1.empty()) {
            returnValue.emplace_back(ret1[0], -1, 1);
        }
        auto ret2 = returnParametricIntersection(edge, Edge(triangle.vertexIndex[2], triangle.vertexIndex[0]));
        if(!ret2.empty()) {
            returnValue.emplace_back(ret2[0], -1, 2);
        }

        if(returnValue.empty()) {
            return {};
        } else {
            return returnValue;
        }
    }

    struct LineCoord {
        float t;
        size_t pointsIndex;
    };

    static bool epsEqual(const float a, const float b, const float EPS = 0.00001f) {
        return abs(a - b) < EPS;
    }

    size_t getIndexOfNewPoint(std::vector<LineCoord>& inserted, float newPt, sf::Vector2f pos, sf::Vector2f dir) {
        sf::Vector2f precise(pos + newPt * dir);
        // Check if we already added this point in this edge run
        for(const LineCoord& oldPt : inserted) {
            if(epsEqual(oldPt.t, newPt)) {
                return oldPt.pointsIndex;
            }
        }
        // Check if such a point already exists in the geometry
        for(size_t i = 0; i < mPoints.size(); ++i) {
            const sf::Vector2f& point = mPoints[i];
            if(epsEqual(precise.x, point.x, 0.51f) && epsEqual(precise.y, point.y, 0.51f)) {
                return i;
            }
        }
        sf::Vector2f rouhedDown(std::floor(precise.x), std::floor(precise.y));
        mPoints.emplace_back(rouhedDown);
        LineCoord newIns{};
        newIns.t = newPt;
        newIns.pointsIndex = mPoints.size() - 1;
        inserted.emplace_back(newIns);
        return newIns.pointsIndex;
    }

    void insertEdge(const Edge& edge) {
        const sf::Vector2f pos = mPoints[edge.first];
        const sf::Vector2f dir = mPoints[edge.second] - mPoints[edge.first];

        std::vector<Triangle> newTriangles;
        newTriangles.reserve(mTriangles.size());

        std::vector<LineCoord> inserted;

        for(const auto& tri : mTriangles) {
            // If triangle doesnt get intersected, copy it over
            auto intersectionWithTri = edgeIntersectsTriangle(edge, tri);
            if(!intersectionWithTri.has_value()) {
                // Insert the triangle
                newTriangles.emplace_back(tri);
                // Insert its potential important edge
                bool contains1 = false;
                bool contains2 = false;
                for(int i = 0; i < 3; ++i) {
                    if(tri.vertexIndex[i] == edge.first) {
                        contains1 = true;
                    }
                    if(tri.vertexIndex[i] == edge.second) {
                        contains2 = true;
                    }
                }
                if(contains1 && contains2) {
                    mImportant.insertEdge(edge);
                }
            } else {  // Triangle gets intersected
                auto intersections = intersectionWithTri.value();
                assert(intersections.size() == 2);

                // Check if we already inserted the 2 points, if not, insert to mPoints, else retrieve the index
                size_t newPtIndex0 = std::numeric_limits<size_t>::max();
                size_t newPtIndex1 = std::numeric_limits<size_t>::max();
                if(intersections[0].vertexIntersected == -1) {
                    newPtIndex0 = getIndexOfNewPoint(inserted, intersections[0].t, pos, dir);
                }
                if(intersections[1].vertexIntersected == -1) {
                    newPtIndex1 = getIndexOfNewPoint(inserted, intersections[1].t, pos, dir);
                }

                // Do we have a 5 point polygon or 4 point polygon (triangle intersected vertex-side, or side-side)?
                if(intersections[0].edgeIntersected != -1 && intersections[1].edgeIntersected != -1) {
                    // 5 point polygon
                    assert(newPtIndex0 != std::numeric_limits<size_t>::max());
                    assert(newPtIndex1 != std::numeric_limits<size_t>::max());

                    bool invalid = false;
                    for(size_t i = 0; i < 3; ++i) {
                        if(newPtIndex0 == tri.vertexIndex[i] || newPtIndex1 == tri.vertexIndex[i]) {
                            invalid = true;
                        }
                    }
                    if(invalid) {
                        continue;
                    }

                    if(intersections[0].edgeIntersected + intersections[1].edgeIntersected == 1) {  // 0 and 1
                        size_t indexEdge0 = std::numeric_limits<size_t>::max();
                        size_t indexEdge1 = std::numeric_limits<size_t>::max();
                        if(intersections[0].edgeIntersected == 0) {
                            indexEdge0 = newPtIndex0;
                            indexEdge1 = newPtIndex1;
                        } else {
                            indexEdge0 = newPtIndex1;
                            indexEdge1 = newPtIndex0;
                        }
                        assert(indexEdge0 != std::numeric_limits<size_t>::max() &&
                               indexEdge1 != std::numeric_limits<size_t>::max());

                        // Register the important edge
                        mImportant.insertEdge(Edge(indexEdge0, indexEdge1));

                        // triangle triangle.vertex==1, intersect.edge==0, intersect.edge==1
                        newTriangles.emplace_back(tri.vertexIndex[1], indexEdge0, indexEdge1);
                        assert(tri.vertexIndex[1] != indexEdge0 && indexEdge0 != indexEdge1 &&
                               indexEdge1 != tri.vertexIndex[1]);

                        // triangle triangle.vertex==2, intersect.edge==0, triangle.vertex==0
                        newTriangles.emplace_back(tri.vertexIndex[2], indexEdge0, tri.vertexIndex[0]);
                        assert(tri.vertexIndex[2] != indexEdge0 && indexEdge0 != tri.vertexIndex[0] &&
                               tri.vertexIndex[0] != tri.vertexIndex[2]);

                        // triangle triangle.vertex==2, intersect.edge==1, intersect.edge==0
                        newTriangles.emplace_back(tri.vertexIndex[2], indexEdge1, indexEdge0);
                        assert(tri.vertexIndex[2] != indexEdge1 && indexEdge1 != indexEdge0 &&
                               indexEdge0 != tri.vertexIndex[2]);
                    }
                    if(intersections[0].edgeIntersected + intersections[1].edgeIntersected == 2) {  // 2 and 0
                        size_t indexEdge0 = std::numeric_limits<size_t>::max();
                        size_t indexEdge2 = std::numeric_limits<size_t>::max();
                        if(intersections[0].edgeIntersected == 0) {
                            indexEdge0 = newPtIndex0;
                            indexEdge2 = newPtIndex1;
                        } else {
                            indexEdge0 = newPtIndex1;
                            indexEdge2 = newPtIndex0;
                        }
                        assert(indexEdge0 != std::numeric_limits<size_t>::max() &&
                               indexEdge2 != std::numeric_limits<size_t>::max());

                        // Register the important edge
                        mImportant.insertEdge(Edge(indexEdge0, indexEdge2));

                        // triangle triangle.vertex==0,intersect.edge==2,intersect.edge==0
                        newTriangles.emplace_back(tri.vertexIndex[0], indexEdge2, indexEdge0);
                        assert(tri.vertexIndex[0] != indexEdge2 && indexEdge2 != indexEdge0 &&
                               tri.vertexIndex[0] != indexEdge0);

                        // triangle triangle.vertex==1, intersect.edge==2, triangle.vertex==2
                        newTriangles.emplace_back(tri.vertexIndex[1], indexEdge2, tri.vertexIndex[2]);
                        assert(tri.vertexIndex[1] != indexEdge2 && indexEdge2 != tri.vertexIndex[2] &&
                               tri.vertexIndex[1] != tri.vertexIndex[2]);

                        // triangle triangle.vertex==1, intersect.edge==0, intersect.edge==2
                        newTriangles.emplace_back(tri.vertexIndex[1], indexEdge0, indexEdge2);
                        assert(tri.vertexIndex[1] != indexEdge0 && indexEdge0 != indexEdge2 &&
                               tri.vertexIndex[1] != indexEdge2);
                    }
                    if(intersections[0].edgeIntersected + intersections[1].edgeIntersected == 3) {  // 1 and 2
                        size_t indexEdge1 = std::numeric_limits<size_t>::max();
                        size_t indexEdge2 = std::numeric_limits<size_t>::max();
                        if(intersections[0].edgeIntersected == 1) {
                            indexEdge1 = newPtIndex0;
                            indexEdge2 = newPtIndex1;
                        } else {
                            indexEdge1 = newPtIndex1;
                            indexEdge2 = newPtIndex0;
                        }
                        assert(indexEdge1 != std::numeric_limits<size_t>::max() &&
                               indexEdge2 != std::numeric_limits<size_t>::max());

                        // Register the important edge
                        mImportant.insertEdge(Edge(indexEdge1, indexEdge2));

                        // triangle triangle.vertex==2,intersect.edge==1, intersect.edge==2
                        newTriangles.emplace_back(tri.vertexIndex[2], indexEdge1, indexEdge2);
                        assert(tri.vertexIndex[2] != indexEdge1 && indexEdge1 != indexEdge2 &&
                               tri.vertexIndex[2] != indexEdge2);
                        // triangle triangle.vertex==1, triangle.vertex==0, intersect.edge==2
                        newTriangles.emplace_back(tri.vertexIndex[1], tri.vertexIndex[0], indexEdge2);
                        assert(tri.vertexIndex[1] != tri.vertexIndex[0] && tri.vertexIndex[0] != indexEdge2 &&
                               tri.vertexIndex[1] != indexEdge2);
                        // triangle triangle.vertex==1, intersect.edge==2, intersect.edge==1
                        newTriangles.emplace_back(tri.vertexIndex[1], indexEdge2, indexEdge1);
                        assert(tri.vertexIndex[1] != indexEdge2 && indexEdge2 != indexEdge1 &&
                               tri.vertexIndex[1] != indexEdge1);
                    }
                } else {
                    // 4 point polygon
                    Intersection vert;
                    size_t vertInd = std::numeric_limits<size_t>::max();
                    Intersection edg;
                    size_t edgInd = std::numeric_limits<size_t>::max();

                    if(intersections[0].edgeIntersected == -1) {
                        vert = intersections[0];
                        assert(newPtIndex0 == std::numeric_limits<size_t>::max());
                        edg = intersections[1];
                        edgInd = newPtIndex1;
                        assert(intersections[1].edgeIntersected != -1 && intersections[0].vertexIntersected != -1);
                    } else if(intersections[1].edgeIntersected == -1) {
                        vert = intersections[1];
                        assert(newPtIndex1 == std::numeric_limits<size_t>::max());
                        edg = intersections[0];
                        edgInd = newPtIndex0;
                        assert(intersections[0].edgeIntersected != -1 && intersections[1].vertexIntersected != -1);
                    } else {
                        assert(false);
                    }

                    assert(vert.vertexIntersected != -1);
                    assert(edg.edgeIntersected != -1);


                    // Indices to mPoints for all 4 points
                    vertInd = tri.vertexIndex[vert.vertexIntersected];
                    assert(vertInd != std::numeric_limits<size_t>::max());
                    assert(edgInd != std::numeric_limits<size_t>::max());

                    // Register the important edge
                    mImportant.insertEdge(Edge(vertInd, edgInd));

                    // Check if we split important edge
                    Edge splitEdge;
                    if(edg.edgeIntersected == 0) {
                        splitEdge = Edge(tri.vertexIndex[0], tri.vertexIndex[1]);
                    } else if(edg.edgeIntersected == 1) {
                        splitEdge = Edge(tri.vertexIndex[1], tri.vertexIndex[2]);
                    } else if(edg.edgeIntersected == 2) {
                        splitEdge = Edge(tri.vertexIndex[2], tri.vertexIndex[0]);
                    } else {
                        splitEdge = Edge(-1, -1);
                        assert(false);
                    }
                    const bool importantEdgeSplit = mImportant.isImportant(splitEdge);
                    if(importantEdgeSplit) {
                        mImportant.insertEdge(Edge(splitEdge.first, edgInd));
                        mImportant.insertEdge(Edge(edgInd, splitEdge.second));
                        mImportant.removeEdge(splitEdge);
                    }

                    bool invalid = false;
                    for(size_t i = 0; i < 3; ++i) {
                        if(edgInd == tri.vertexIndex[i]) {
                            invalid = true;
                        }
                    }

                    // The 4 new Indices are:
                    // vertInd - shared by both triangles
                    // edgInd - shared by both triangles
                    if(!invalid) {
                        if(vert.vertexIntersected == 0)  // vertex shared is 0 {
                        {
                            newTriangles.emplace_back(vertInd, tri.vertexIndex[1], edgInd);
                            assert(vertInd != tri.vertexIndex[1] && tri.vertexIndex[1] != edgInd && edgInd != vertInd);

                            newTriangles.emplace_back(vertInd, edgInd, tri.vertexIndex[2]);
                            assert(vertInd != edgInd && edgInd != tri.vertexIndex[2] && tri.vertexIndex[2] != vertInd);
                        } else if(vert.vertexIntersected == 1)  // vertex shared is 1
                        {
                            newTriangles.emplace_back(vertInd, tri.vertexIndex[2], edgInd);
                            assert(vertInd != tri.vertexIndex[2] && tri.vertexIndex[2] != edgInd && edgInd != vertInd);

                            newTriangles.emplace_back(edgInd, tri.vertexIndex[0], vertInd);
                            assert(edgInd != tri.vertexIndex[0] && tri.vertexIndex[0] != vertInd && vertInd != edgInd);
                        } else if(vert.vertexIntersected == 2)  // vertex shared is 2
                        {
                            newTriangles.emplace_back(vertInd, tri.vertexIndex[0], edgInd);
                            assert(vertInd != tri.vertexIndex[0] && tri.vertexIndex[0] != edgInd && vertInd != edgInd);

                            newTriangles.emplace_back(edgInd, tri.vertexIndex[1], vertInd);
                            assert(edgInd != tri.vertexIndex[1] && tri.vertexIndex[1] != vertInd && vertInd != edgInd);
                        } else {
                            assert(false);
                        }
                    } else {
                        newTriangles.emplace_back(tri);
                    }
                }
            }
        }

        mTriangles = std::move(newTriangles);
    }

    /// Constrained Delaunay Triangulation

    bool oneCdtPass() {
        const std::array<std::pair<size_t, size_t>, 3> pairs = {std::pair<size_t, size_t>(0, 1), {1, 2}, {2, 0}};
        for(size_t i = 0; i < mTriangles.size(); ++i) {
            for(const auto& pair : pairs) {
                const auto& triVertices = mTriangles[i].vertexIndex;
                const Edge currentEdge(triVertices[pair.first], triVertices[pair.second]);
                if(mImportant.isImportant(currentEdge)) {
                    continue;
                }
                auto flipped = flip(i, pair.first, pair.second);
                if(!flipped.empty()) {
                    return true;
                }
            }
        }
        return false;
    }

    void cdt() {
        bool changed = false;
        changed = oneCdtPass();

        if(changed) {
            std::cout << "Changed.\n";
        } else {
            std::cout << "Not changed.";
        }
    }

    void cdt_all() {
        bool changed = false;
        std::cout << "CDT starting.\n";
        do {
            changed = oneCdtPass();
        } while(changed);
        std::cout << "CDT finished.\n";
    }
};
