#include "graph.hpp"

void Graph::update(const rclcpp::Node::SharedPtr & node)
{
  for (const auto & node : node->get_node_names())
  {
    nodes[node] = std::make_unique<GraphNode>(node);
  }

  for (const auto & topic : node->get_topic_names_and_types())
  {
    topics[topic.first] = std::make_unique<GraphTopic>(topic.first);

    for (const auto & endpoint : node->get_publishers_info_by_topic(topic.first))
    {
      auto link = std::make_unique<GraphLink>(topic.first, endpoint);
      link->is_publish = true;
      links.push_back(std::move(link));
    }
    for (const auto & endpoint : node->get_subscriptions_info_by_topic(topic.first))
    {
      auto link = std::make_unique<GraphLink>(topic.first, endpoint);
      link->is_publish = false;
      links.push_back(std::move(link));
    }
  }
}

GraphTopic::GraphTopic(const std::string & name) : name(name)
{
}

GraphNode::GraphNode(const std::string & name) : name(name)
{
}

GraphLink::GraphLink(const std::string & topic, const rclcpp::TopicEndpointInfo & endpoint) : topic(topic)
{
  node = endpoint.node_namespace() + "/" + endpoint.node_name();
  type = endpoint.topic_type();

  switch (endpoint.qos_profile().get_rmw_qos_profile().reliability)
  {
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
      reliability = "default";
      break;
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      reliability = "reliable";
      break;
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      reliability = "best_effort";
      break;
    default:
      reliability = "unknown";
      break;
  }

  switch (endpoint.qos_profile().get_rmw_qos_profile().reliability)
  {
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
      durability = "default";
      break;
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      durability = "volatile";
      break;
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      durability = "transient_local";
      break;
    default:
      durability = "unknown";
      break;
  }
}
