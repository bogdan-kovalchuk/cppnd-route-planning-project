#include "gtest/gtest.h"
#include <fstream>
#include <iostream>
#include <optional>
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
