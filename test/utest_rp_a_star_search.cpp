#include "gtest/gtest.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>
#include "../src/route_model.h"
#include "../src/route_planner.h"


static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

std::vector<std::byte> ReadOSMData(const std::string &path) {
    std::vector<std::byte> osm_data;
    auto data = ReadFile(path);
    if( !data ) {
        std::cout << "Failed to read OSM data." << std::endl;
    } else {
        osm_data = std::move(*data);
    }
    return osm_data;
}

// Converts an inline OSM XML fixture into the byte buffer the Model
// constructor expects, so edge cases can be exercised without map.osm.
static std::vector<std::byte> ToBytes(const std::string &text) {
    std::vector<std::byte> bytes(text.size());
    for (size_t i = 0; i < text.size(); i++) {
        bytes[i] = static_cast<std::byte>(text[i]);
    }
    return bytes;
}

//--------------------------------//
//   Beginning RoutePlanner Tests.
//--------------------------------//

class RoutePlannerTest : public ::testing::Test {
  protected:
    std::string osm_data_file = "../map.osm";
    std::vector<std::byte> osm_data = ReadOSMData(osm_data_file);
    RouteModel model{osm_data};
    RoutePlanner route_planner{model, 10, 10, 90, 90};
    
    // Construct start_node and end_node as in the model.
    float start_x = 0.1;
    float start_y = 0.1;
    float end_x = 0.9;
    float end_y = 0.9;
    RouteModel::Node* start_node = &model.FindClosestNode(start_x, start_y);
    RouteModel::Node* end_node = &model.FindClosestNode(end_x, end_y);

    // Construct another node in the middle of the map for testing.
    float mid_x = 0.5;
    float mid_y = 0.5;
    RouteModel::Node* mid_node = &model.FindClosestNode(mid_x, mid_y);
};


TEST_F(RoutePlannerTest, TestNodeDistanceHelper) {
    EXPECT_FLOAT_EQ(start_node->distance(*start_node), 0.0f);
    float d1 = start_node->distance(*end_node);
    float d2 = end_node->distance(*start_node);
    EXPECT_FLOAT_EQ(d1, d2);
    EXPECT_GT(d1, 0.0f);
}

TEST_F(RoutePlannerTest, TestCompareFunction) {
    start_node->g_value = 1.0f;
    start_node->h_value = 2.0f;
    end_node->g_value = 0.5f;
    end_node->h_value = 0.5f;
    EXPECT_TRUE(Compare(start_node, end_node));
    EXPECT_FALSE(Compare(end_node, start_node));
}

// Test the CalculateHValue method.
TEST_F(RoutePlannerTest, TestCalculateHValue) {
    EXPECT_FLOAT_EQ(route_planner.CalculateHValue(start_node), 1.1329799);
    EXPECT_FLOAT_EQ(route_planner.CalculateHValue(end_node), 0.0f);
    EXPECT_FLOAT_EQ(route_planner.CalculateHValue(mid_node), 0.58903033);
}



// Test the AddNeighbors method.
bool NodesSame(RouteModel::Node* a, RouteModel::Node* b) { return a == b; }
TEST_F(RoutePlannerTest, TestAddNeighbors) {
    route_planner.AddNeighbors(start_node);

    // Correct h and g values for the neighbors of start_node.
    std::vector<float> start_neighbor_g_vals{0.10671431, 0.082997195, 0.051776856, 0.055291083};
    std::vector<float> start_neighbor_h_vals{1.1828455, 1.0998145, 1.0858033, 1.1831238};
    auto neighbors = start_node->neighbors;
    EXPECT_EQ(neighbors.size(), 4);

    // Check results for each neighbor.
    for (int i = 0; i < neighbors.size(); i++) {
        EXPECT_PRED2(NodesSame, neighbors[i]->parent, start_node);
        EXPECT_FLOAT_EQ(neighbors[i]->g_value, start_neighbor_g_vals[i]);
        EXPECT_FLOAT_EQ(neighbors[i]->h_value, start_neighbor_h_vals[i]);
        EXPECT_EQ(neighbors[i]->visited, true);
    }
}


// Test the ConstructFinalPath method.
TEST_F(RoutePlannerTest, TestConstructFinalPath) {
    // Construct a path.
    mid_node->parent = start_node;
    end_node->parent = mid_node;
    std::vector<RouteModel::Node> path = route_planner.ConstructFinalPath(end_node);

    // Test the path.
    EXPECT_EQ(path.size(), 3);
    EXPECT_FLOAT_EQ(start_node->x, path.front().x);
    EXPECT_FLOAT_EQ(start_node->y, path.front().y);
    EXPECT_FLOAT_EQ(end_node->x, path.back().x);
    EXPECT_FLOAT_EQ(end_node->y, path.back().y);
}


TEST_F(RoutePlannerTest, TestConstructFinalPathDistance) {
    mid_node->parent = start_node;
    end_node->parent = mid_node;
    float expected_dist = (mid_node->distance(*start_node) + end_node->distance(*mid_node)) * model.MetricScale();
    std::vector<RouteModel::Node> path = route_planner.ConstructFinalPath(end_node);
    EXPECT_FLOAT_EQ(route_planner.GetDistance(), expected_dist);
}

// Test the AStarSearch method.
TEST_F(RoutePlannerTest, TestFindClosestNodeValid) {
    RouteModel::Node &node = model.FindClosestNode(0.5f, 0.5f);
    EXPECT_GE(node.x, 0.0f);
    EXPECT_LE(node.x, 1.0f);
    EXPECT_GE(node.y, 0.0f);
    EXPECT_LE(node.y, 1.0f);
}

TEST_F(RoutePlannerTest, TestAddNeighborsVisitedFlag) {
    route_planner.AddNeighbors(mid_node);
    for (auto neighbor : mid_node->neighbors) {
        EXPECT_TRUE(neighbor->visited);
    }
}

TEST_F(RoutePlannerTest, TestAStarPathConnectivity) {
    route_planner.AStarSearch();
    EXPECT_GE(model.path.size(), 2);
    EXPECT_GT(route_planner.GetDistance(), 0.0f);
}

TEST_F(RoutePlannerTest, TestAStarDistanceMatchesPath) {
    route_planner.AStarSearch();
    float manual_dist = 0.0f;
    for (size_t i = 1; i < model.path.size(); i++) {
        manual_dist += model.path[i].distance(model.path[i-1]);
    }
    manual_dist *= model.MetricScale();
    EXPECT_FLOAT_EQ(route_planner.GetDistance(), manual_dist);
}

TEST_F(RoutePlannerTest, TestRouteModelNodeCount) {
    EXPECT_EQ(model.SNodes().size(), model.Nodes().size());
    EXPECT_GT(model.SNodes().size(), 0);
}

TEST_F(RoutePlannerTest, TestGValueAccumulation) {
    route_planner.AStarSearch();
    for (size_t i = 1; i < model.path.size(); i++) {
        EXPECT_GE(model.path[i].g_value, model.path[i-1].g_value);
    }
}

TEST_F(RoutePlannerTest, TestMetricScaleInDistance) {
    route_planner.AStarSearch();
    float raw_dist = 0.0f;
    for (size_t i = 1; i < model.path.size(); i++) {
        raw_dist += model.path[i].distance(model.path[i-1]);
    }
    EXPECT_FLOAT_EQ(route_planner.GetDistance(), raw_dist * model.MetricScale());
}

TEST_F(RoutePlannerTest, TestAStarSameStartEnd) {
    RoutePlanner trivial_planner{model, 50, 50, 50, 50};
    trivial_planner.AStarSearch();
    EXPECT_EQ(model.path.size(), 1);
    EXPECT_FLOAT_EQ(trivial_planner.GetDistance(), 0.0f);
}

TEST_F(RoutePlannerTest, TestConstructFinalPathSingleNode) {
    std::vector<RouteModel::Node> path = route_planner.ConstructFinalPath(start_node);
    EXPECT_EQ(path.size(), 1);
    EXPECT_FLOAT_EQ(path[0].x, start_node->x);
    EXPECT_FLOAT_EQ(path[0].y, start_node->y);
    EXPECT_FLOAT_EQ(route_planner.GetDistance(), 0.0f);
}

TEST_F(RoutePlannerTest, TestAddNeighborsGValue) {
    start_node->g_value = 0.0f;
    route_planner.AddNeighbors(start_node);
    for (auto neighbor : start_node->neighbors) {
        float expected_g = start_node->g_value + start_node->distance(*neighbor);
        EXPECT_FLOAT_EQ(neighbor->g_value, expected_g);
    }
}

TEST_F(RoutePlannerTest, TestAStarPathOrdering) {
    route_planner.AStarSearch();
    EXPECT_GE(model.path.size(), 2);
    EXPECT_FLOAT_EQ(model.path.front().x, start_node->x);
    EXPECT_FLOAT_EQ(model.path.front().y, start_node->y);
    EXPECT_FLOAT_EQ(model.path.back().x, end_node->x);
    EXPECT_FLOAT_EQ(model.path.back().y, end_node->y);
}

TEST_F(RoutePlannerTest, TestAStarSearch) {
    route_planner.AStarSearch();
    EXPECT_EQ(model.path.size(), 33);
    RouteModel::Node path_start = model.path.front();
    RouteModel::Node path_end = model.path.back();
    // The start_node and end_node x, y values should be the same as in the path.
    EXPECT_FLOAT_EQ(start_node->x, path_start.x);
    EXPECT_FLOAT_EQ(start_node->y, path_start.y);
    EXPECT_FLOAT_EQ(end_node->x, path_end.x);
    EXPECT_FLOAT_EQ(end_node->y, path_end.y);
    EXPECT_FLOAT_EQ(route_planner.GetDistance(), 873.41565);
}

TEST_F(RoutePlannerTest, TestFValueComputation) {
    start_node->g_value = 1.0f;
    start_node->h_value = 2.0f;
    EXPECT_FLOAT_EQ(FValue(start_node), 3.0f);
}

TEST_F(RoutePlannerTest, TestFValueMatchesCompare) {
    start_node->g_value = 1.0f;
    start_node->h_value = 2.0f;
    end_node->g_value = 0.5f;
    end_node->h_value = 0.5f;
    EXPECT_GT(FValue(start_node), FValue(end_node));
    EXPECT_TRUE(Compare(start_node, end_node));
}

TEST_F(RoutePlannerTest, TestRoadTypeToString) {
    EXPECT_STREQ(Model::RoadTypeToString(Model::Road::Motorway), "motorway");
    EXPECT_STREQ(Model::RoadTypeToString(Model::Road::Footway), "footway");
    EXPECT_STREQ(Model::RoadTypeToString(Model::Road::Invalid), "invalid");
}

TEST_F(RoutePlannerTest, TestPathDistanceEmpty) {
    std::vector<RouteModel::Node> empty_path;
    EXPECT_FLOAT_EQ(PathDistance(empty_path), 0.0f);
}

TEST_F(RoutePlannerTest, TestPathDistanceMatchesSearch) {
    route_planner.AStarSearch();
    float manual = PathDistance(model.path) * model.MetricScale();
    EXPECT_FLOAT_EQ(route_planner.GetDistance(), manual);
}

//------------------------------------------------//
//   Synthetic-map tests (disconnected/edge maps).
//------------------------------------------------//

static const char *kTwoIslandsOsm =
    "<?xml version='1.0'?><osm>"
    "<bounds minlat='0' minlon='0' maxlat='1' maxlon='1'/>"
    "<node id='1' lat='0.10' lon='0.10'/>"
    "<node id='2' lat='0.15' lon='0.15'/>"
    "<node id='3' lat='0.20' lon='0.20'/>"
    "<node id='10' lat='0.60' lon='0.60'/>"
    "<node id='11' lat='0.65' lon='0.65'/>"
    "<node id='20' lat='0.90' lon='0.90'/>"
    "<node id='21' lat='0.95' lon='0.95'/>"
    "<way id='100'><nd ref='1'/><nd ref='2'/><nd ref='3'/>"
    "<tag k='highway' v='residential'/></way>"
    "<way id='101'><nd ref='10'/><nd ref='11'/>"
    "<tag k='highway' v='residential'/></way>"
    "<way id='102'><nd ref='20'/><nd ref='21'/>"
    "<tag k='highway' v='residential'/></way>"
    "</osm>";

// A search that fails to reach the end node must not leave the path
// (or distance) from a previous, successful search in place.
TEST(RoutePlannerSyntheticTest, RepeatedSearchClearsStalePath) {
    RouteModel model(ToBytes(kTwoIslandsOsm));

    RoutePlanner connected_planner(model, 10, 10, 20, 20);
    connected_planner.AStarSearch();
    ASSERT_GE(model.path.size(), 2u);
    ASSERT_GT(connected_planner.GetDistance(), 0.0f);

    RoutePlanner disconnected_planner(model, 60, 60, 90, 90);
    disconnected_planner.AStarSearch();
    EXPECT_TRUE(model.path.empty());
    EXPECT_FLOAT_EQ(disconnected_planner.GetDistance(), 0.0f);
}

static const char *kEmptyOsm =
    "<?xml version='1.0'?><osm>"
    "<bounds minlat='0' minlon='0' maxlat='1' maxlon='1'/>"
    "</osm>";

// A map with valid bounds but no road nodes has nothing to search;
// FindClosestNode must report that instead of indexing an empty vector.
TEST(RoutePlannerSyntheticTest, FindClosestNodeThrowsWhenNoNodes) {
    RouteModel model(ToBytes(kEmptyOsm));
    EXPECT_THROW(model.FindClosestNode(0.5f, 0.5f), std::logic_error);
}

// The Model constructor requires a <bounds> element to establish the
// coordinate frame; a map without one must fail fast rather than build
// a model with an undefined metric scale.
TEST(RoutePlannerSyntheticTest, ModelThrowsWhenBoundsMissing) {
    static const char *kNoBoundsOsm = "<?xml version='1.0'?><osm></osm>";
    EXPECT_THROW(RouteModel model(ToBytes(kNoBoundsOsm)), std::logic_error);
}

// Truncated/unparseable XML must be rejected up front rather than
// silently producing a partially built model.
TEST(RoutePlannerSyntheticTest, ModelThrowsOnMalformedXml) {
    static const char *kMalformedXml = "<osm><bounds minlat='0'";
    EXPECT_THROW(RouteModel model(ToBytes(kMalformedXml)), std::logic_error);
}

// AdjustCoordinates normalizes every node relative to the map bounds and
// scales by the smaller of the projected width/height (MetricScale), so
// the min-bound node must land exactly at the origin and the max-bound
// node must sit exactly on the unit boundary along at least one axis.
TEST(RoutePlannerSyntheticTest, AdjustCoordinatesNormalizesToUnitBox) {
    static const char *kBoundsOsm =
        "<?xml version='1.0'?><osm>"
        "<bounds minlat='0' minlon='0' maxlat='0.01' maxlon='0.02'/>"
        "<node id='1' lat='0' lon='0'/>"
        "<node id='2' lat='0.01' lon='0.02'/>"
        "</osm>";
    RouteModel model(ToBytes(kBoundsOsm));
    ASSERT_EQ(model.Nodes().size(), 2u);

    const auto &min_corner = model.Nodes()[0];
    const auto &max_corner = model.Nodes()[1];

    EXPECT_FLOAT_EQ(min_corner.x, 0.0f);
    EXPECT_FLOAT_EQ(min_corner.y, 0.0f);
    EXPECT_GT(model.MetricScale(), 0.0);
    EXPECT_GE(max_corner.x, 0.0);
    EXPECT_GE(max_corner.y, 0.0);
    EXPECT_TRUE(std::abs(max_corner.x - 1.0) < 1e-4 || std::abs(max_corner.y - 1.0) < 1e-4);
}

// A <nd ref> that does not match any parsed node id (e.g. a way that
// references a node outside the extracted bounding box) must be dropped
// from the way instead of crashing or inserting a bogus index.
TEST(RoutePlannerSyntheticTest, DanglingNodeRefIsDroppedFromWay) {
    static const char *kDanglingRefOsm =
        "<?xml version='1.0'?><osm>"
        "<bounds minlat='0' minlon='0' maxlat='1' maxlon='1'/>"
        "<node id='1' lat='0.10' lon='0.10'/>"
        "<node id='2' lat='0.20' lon='0.20'/>"
        "<way id='100'><nd ref='1'/><nd ref='999'/><nd ref='2'/>"
        "<tag k='highway' v='residential'/></way>"
        "</osm>";
    RouteModel model(ToBytes(kDanglingRefOsm));
    ASSERT_EQ(model.Ways().size(), 1u);
    EXPECT_EQ(model.Ways()[0].nodes.size(), 2u);
    EXPECT_EQ(model.Ways()[0].nodes[0], 0);
    EXPECT_EQ(model.Ways()[0].nodes[1], 1);
}

// A way with a single node has no other node to link to; FindNeighbor's
// self-exclusion (distance != 0) must prevent it from becoming its own
// neighbor rather than looping back on itself.
TEST(RoutePlannerSyntheticTest, SingleNodeWayHasNoNeighbors) {
    static const char *kSingleNodeWayOsm =
        "<?xml version='1.0'?><osm>"
        "<bounds minlat='0' minlon='0' maxlat='1' maxlon='1'/>"
        "<node id='1' lat='0.10' lon='0.10'/>"
        "<way id='100'><nd ref='1'/>"
        "<tag k='highway' v='residential'/></way>"
        "</osm>";
    RouteModel model(ToBytes(kSingleNodeWayOsm));
    ASSERT_EQ(model.SNodes().size(), 1u);
    model.SNodes()[0].FindNeighbors();
    EXPECT_TRUE(model.SNodes()[0].neighbors.empty());
}
