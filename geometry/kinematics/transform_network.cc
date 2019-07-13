#include "geometry/kinematics/transform_network.hh"

#include "logging/assert.hh"

#include <algorithm>
#include <set>
#include <stack>

namespace geometry {

void TransformNetwork::add_edge(const std::string& source,
                                const std::string& destination,
                                const SE3& source_from_destination) {
  JASSERT_EQ(edges_from_node_tag_[source].count(destination),
             0u,
             "Destination already in the graph!");
  edges_from_node_tag_[source][destination] =
      Edge{.source_from_destination = source_from_destination};
  edges_from_node_tag_[destination][source] =
      Edge{.source_from_destination = source_from_destination.inverse()};
}

void TransformNetwork::update_edge(const std::string& source,
                                   const std::string& destination,
                                   const SE3& source_from_destination) {
  // Uh oh! Remind you of any other members!?
  edges_from_node_tag_[source][destination] =
      Edge{.source_from_destination = source_from_destination};
  edges_from_node_tag_[destination][source] =
      Edge{.source_from_destination = source_from_destination.inverse()};
}

SE3 TransformNetwork::find_source_from_destination(const std::string& source,
                                                   const std::string& destination) const {
  JASSERT_NE(edges_from_node_tag_.count(source), 0u, "Not in the network");
  JASSERT_NE(edges_from_node_tag_.count(destination), 0u, "Not in the network");
  std::stack<std::string> to_visit;

  std::stack<std::string> path;
  std::set<std::string> visited;
  to_visit.push(source);

  while (!to_visit.empty()) {
    const auto parent = to_visit.top();
    to_visit.pop();

    while (!path.empty() && edges_from_node_tag_.at(parent).count(path.top()) == 0u) {
      path.pop();
    }

    path.push(parent);

    if (parent == destination) {
      break;
    }

    bool any_added = false;
    for (const auto& child : edges_from_node_tag_.at(parent)) {
      if (visited.count(child.first) == 0u) {
        any_added = true;
        to_visit.push(child.first);
        visited.insert(child.first);
      }
    }

    if (!any_added) {
      path.pop();
      continue;
    }
  }

  SE3 full_source_from_destination;
  while (path.size() > 1u) {
    const std::string child = path.top();
    path.pop();
    const std::string parent = path.top();

    const SE3 parent_from_child =
        edges_from_node_tag_.at(parent).at(child).source_from_destination;

    full_source_from_destination = parent_from_child * full_source_from_destination;
  }

  return full_source_from_destination;
}

// Cannot override edges
void TransformNetwork::insert(const TransformNetwork& other) {
  for (const auto& node_pair : other.edges_from_node_tag_) {
    const auto& node = node_pair.first;
    if (edges_from_node_tag_.count(node) == 0u) {
      edges_from_node_tag_[node] = other.edges_from_node_tag_.at(node);
    } else {
      const auto& our_edges = edges_from_node_tag_.at(node);

      for (const auto& other_edge : node_pair.second) {
        const auto& other_destination = other_edge.first;
        JASSERT_EQ(our_edges.count(other_destination),
                   0u,
                   "Edge updates not permitted on transform network insertion");
        edges_from_node_tag_.at(node).insert(other_edge);
      }
    }
  }
}

}  // namespace geometry