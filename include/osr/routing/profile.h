#pragma once

#include <concepts>
#include <string_view>

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

template <typename Node, typename NodeValue>
concept IsNodeBase = requires(Node node, NodeValue value) {
  { node.get_node() } -> std::same_as<NodeValue>;
};

template <typename Node, typename NodeKey>
concept IsNode = IsNodeBase<Node, osr::node_idx_t> && requires(Node node) {
  { node.get_key() } -> std::same_as<NodeKey>;
};

template <typename Label, typename Node>
concept IsLabel = IsNodeBase<Label, Node> && requires(Label label) {
  { label.cost() } -> std::same_as<osr::cost_t>;
};

template <typename Entry, typename Node, typename Label>
concept IsEntry =
    requires(Entry entry, Node node, Label label, osr::cost_t cost) {
      { entry.pred(node) } -> std::same_as<std::optional<Node>>;
      { entry.cost(node) } -> std::same_as<osr::cost_t>;
      { entry.update(label, node, cost, node) } -> std::same_as<bool>;
    };

template <typename T>
concept IsProfile =
    IsNode<typename T::node, typename T::key> &&
    IsLabel<typename T::label, typename T::node> &&
    IsEntry<typename T::entry, typename T::node, typename T::label> &&
    requires(T profile) {
      T::resolve_start_node(osr::ways::routing(), osr::way_idx_t(),
                            osr::node_idx_t(), osr::level_t(), osr::direction(),
                            [](T::node) {});
      T::resolve_all(osr::ways::routing(), osr::node_idx_t(), osr::level_t(),
                     [](T::node) {});
      // T::adjacent<osr::direction(), bool()>(
      //     osr::ways::routing(), typename T::node(),
      //     new osr::bitvec<osr::node_idx_t>(), nullptr, []() {});
      // [](auto Dir, auto B, auto Fn) {
      //   T::adjacent<Dir, B, Fn>(osr::ways::routing(), typename T::node(),
      //                           new osr::bitvec<osr::node_idx_t>(), nullptr,
      //                           Fn);
      // };
      // [](auto Dir, auto B, auto Fn) {
      //   T::adjacent<Dir, B, Fn>(osr::ways::routing(), typename T::entry(),
      //                           new osr::bitvec<osr::node_idx_t>(), nullptr,
      //                           Fn);
      // };
      // T::adjacent<osr::direction(), bool()>(
      //     osr::ways::routing(), typename T::node(), nullptr, nullptr, []()
      //     {});
    };

enum class search_profile : std::uint8_t {
  kFoot,
  kWheelchair,
  kBike,
  kCar,
  kCarParking,
  kCarParkingWheelchair,
  kBikeSharing,
};

search_profile to_profile(std::string_view);

std::string_view to_str(search_profile);

}  // namespace osr
