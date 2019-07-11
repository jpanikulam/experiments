#pragma once

#include "sophus.hh"

#include <map>
#include <string>

namespace geometry {

// TODO: Generic graph
//
// - Use for viewer
// - Use for robots
//
class TransformNetwork {
 public:
  struct Edge {
    SE3 source_from_destination;
  };

  void add_edge(const std::string& source,
                const std::string& destination,
                const SE3& source_from_destination);

  using EdgeMap = std::map<std::string, Edge>;
  const std::map<std::string, EdgeMap>& edges_from_node_tag() const {
    return edges_from_node_tag_;
  }

  // DFS
  // TODO: Cache some transitive connections
  SE3 source_from_destination(const std::string& source,
                               const std::string& destination) const;

 private:
  std::map<std::string, EdgeMap> edges_from_node_tag_;
};

}  // namespace geometry