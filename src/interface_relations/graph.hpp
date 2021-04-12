#include <memory>
#include <vector>
#include <map>
#include "rclcpp/rclcpp.hpp"

namespace tkgism_ros_tools {

struct Graph;
struct GraphTopic;
struct GraphNode;
struct GraphLink;

struct Graph
{
  void update(const rclcpp::Node::SharedPtr & node);
  void print_topics();

  std::map<std::string, std::unique_ptr<GraphNode>> nodes;
  std::map<std::string, std::unique_ptr<GraphTopic>> topics;
  std::vector<std::unique_ptr<GraphLink>> links;
};

struct GraphTopic
{
  GraphTopic(const std::string & name);

  std::string name;
  std::vector<GraphLink *> pub_links;
  std::vector<GraphLink *> sub_links;
  // std::vector<GraphNode *> src_nodes;
  // std::vector<GraphNode *> dst_nodes;
};

struct GraphNode
{
  GraphNode(const std::string & name);

  std::string name;
  std::vector<GraphLink *> pub_links;
  std::vector<GraphLink *> sub_links;
  // std::vector<GraphTopic *> pub_topics;
  // std::vector<GraphTopic *> sub_topics;
};

struct GraphLink
{
  GraphLink(const std::string & topic, const rclcpp::TopicEndpointInfo & endpoint);

  std::string topic;
  std::string node;
  std::string type;
  std::string reliability;
  std::string durability;
  bool is_publish;
};

}  // namespace tkgism_ros_tools
