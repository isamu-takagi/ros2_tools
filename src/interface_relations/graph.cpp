#include "graph.hpp"
#include <iostream>
#include <ios>
#include <iomanip>

namespace tkgism_ros_tools {

void Graph::update(const rclcpp::Node::SharedPtr & node)
{
  // list nodes
  for (const auto & node : node->get_node_names())
  {
    nodes[node] = std::make_unique<GraphNode>(node);
  }

  // list topics
  for (const auto & topic : node->get_topic_names_and_types())
  {
    topics[topic.first] = std::make_unique<GraphTopic>(topic.first);
  }

  // list links
  for (const auto & topic : topics)
  {
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

  // connect links
  for (const auto& link : links)
  {
    if (nodes.count(link->node))
    {
      link->is_publish ? nodes[link->node]->pub_links.push_back(link.get()) : nodes[link->node]->sub_links.push_back(link.get());
    }
    else
    {
      std::cerr << "unkown node: " << link->node << std::endl;
    }

    if (topics.count(link->topic))
    {
      link->is_publish ? topics[link->topic]->pub_links.push_back(link.get()) : topics[link->topic]->sub_links.push_back(link.get());
    }
    else
    {
      std::cerr << "unkown topic: " << link->topic << std::endl;
    }
  }
}

void Graph::print_topics()
{
  std::size_t n_width = 0;
  std::size_t t_width = 0;
  std::size_t r_width = 0;
  std::size_t d_width = 0;

  for (const auto& link : links)
  {
    if ((link->topic == "/parameter_events") || (link->topic == "/rosout")) { continue; }

    n_width = std::max(n_width, link->node.length());
    t_width = std::max(t_width, link->type.length());
    r_width = std::max(r_width, link->reliability.length());
    d_width = std::max(d_width, link->durability.length());
  }

  auto print_link = [=](const GraphLink * link, const std::string & direction)
  {
    std::cout << std::left << "  " << std::setw(5) << direction;
    std::cout << std::setw(n_width + 2) << link->node;
    std::cout << std::setw(t_width + 2) << link->type;
    std::cout << std::setw(r_width + 2) << link->reliability;
    std::cout << std::setw(d_width + 2) << link->durability;
    std::cout << std::endl;
  };

  for (const auto& [name, topic] : topics)
  {
    if ((name == "/parameter_events") || (name == "/rosout")) { continue; }

    std::cout << name << std::endl;
    for(const auto& link : topic->pub_links)
    {
      print_link(link, "from");
    }
    for(const auto& link : topic->sub_links)
    {
      print_link(link, "to");
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
  node = endpoint.node_namespace();
  node = node + (node == "/" ? "" : "/") + endpoint.node_name();
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

}  // namespace tkgism_ros_tools
