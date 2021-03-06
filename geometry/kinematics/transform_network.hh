#pragma once

#include "sophus.hh"

#include <map>
#include <string>

namespace geometry {

// TODO: Generic graph
//
// - Use for viewer
// - Use for robots
// - Add support for transforming:
//   - Velocities and higher derivatives
//   - Angular velocities and higher derivatives
class TransformNetwork {
 public:
  struct Edge {
    SE3 source_from_destination;
  };

  // Add, check that the edge is not already present
  void add_edge(const std::string& source,
                const std::string& destination,
                const SE3& source_from_destination);

  // Update, do not check if the edge is already present (Create it if it is not)
  void update_edge(const std::string& source,
                   const std::string& destination,
                   const SE3& source_from_destination);

  // DFS
  // TODO: Cache some transitive connections
  SE3 find_source_from_destination(const std::string& source,
                                   const std::string& destination) const;

  template <typename T>
  T transform_a_from_b(const std::string& a,
                       const std::string& b,
                       const T& object_destination_frame) const {
    const SE3 source_from_destination = find_source_from_destination(a, b);
    return source_from_destination * object_destination_frame;
  }

  // Insert an *entire* other transform network
  // Coming soon: Update! Permit overrides of existing edges
  void insert(const TransformNetwork& other);

  // Coming soon!
  // std::optional<SE3> direct_lookup(const std::string& source,
  //                                  const std::string& destination) const;

  using EdgeMap = std::map<std::string, Edge>;
  const std::map<std::string, EdgeMap>& edges_from_node_tag() const {
    return edges_from_node_tag_;
  }

 private:
  std::map<std::string, EdgeMap> edges_from_node_tag_;

  // Coming soon!
  // mutable std::map<std::string, EdgeMap> lookup_cache_;
};

}  // namespace geometry