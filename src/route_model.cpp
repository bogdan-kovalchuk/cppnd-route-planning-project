#include "route_model.h"
#include <algorithm>
#include <functional>
#include <iostream>
#include <stdexcept>

static bool HasTraversableSegment(const Model::Way &way)
{
    return std::adjacent_find(way.nodes.begin(), way.nodes.end(),
                              std::not_equal_to<int>()) != way.nodes.end();
}

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    // Create RouteModel nodes.
    int counter = 0;
    for (Model::Node node : this->Nodes()) {
        m_Nodes.emplace_back(Node(counter, this, node));
        counter++;
    }
    CreateNodeToRoadHashmap();
}


void RouteModel::CreateNodeToRoadHashmap() {
    for (const Model::Road &road : Roads()) {
        const auto &way = Ways()[road.way];
        if (road.type == Model::Road::Type::Footway || !HasTraversableSegment(way))
            continue;

        for (int node_idx : way.nodes) {
            if (node_to_road.find(node_idx) == node_to_road.end()) {
                node_to_road[node_idx] = std::vector<const Model::Road *> ();
            }
            node_to_road[node_idx].push_back(&road);
        }
    }
}


void RouteModel::Node::FindNeighbors() {
    neighbors.clear();
    auto roads = parent_model->node_to_road.find(index);
    if (roads == parent_model->node_to_road.end())
        return;

    auto add_neighbor = [this](int node_index) {
        if (node_index == index)
            return;
        Node &node = parent_model->SNodes()[node_index];
        if (!node.visited &&
            std::find(neighbors.begin(), neighbors.end(), &node) == neighbors.end())
            neighbors.emplace_back(&node);
    };

    for (const auto *road : roads->second) {
        const auto &nodes = parent_model->Ways()[road->way].nodes;
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (nodes[i] != index)
                continue;
            if (i > 0)
                add_neighbor(nodes[i - 1]);
            if (i + 1 < nodes.size())
                add_neighbor(nodes[i + 1]);
        }
    }
}


RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    if (!std::isfinite(x) || !std::isfinite(y))
        throw std::invalid_argument("closest-node coordinates must be finite");
    if (SNodes().empty())
        throw std::logic_error("RouteModel has no nodes to search");

    Node input;
    input.x = x;
    input.y = y;

    float min_dist = std::numeric_limits<float>::max();
    float dist;
    Node *closest_node = nullptr;

    for (const Model::Road &road : Roads()) {
        const auto &way = Ways()[road.way];
        if (road.type == Model::Road::Type::Footway || !HasTraversableSegment(way))
            continue;

        for (int node_idx : way.nodes) {
            dist = input.distance(SNodes()[node_idx]);
            if (dist < min_dist) {
                closest_node = &SNodes()[node_idx];
                min_dist = dist;
            }
        }
    }

    if (closest_node == nullptr)
        throw std::logic_error("RouteModel has no routable nodes to search");

    return *closest_node;
}
