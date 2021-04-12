#include "window.hpp"
#include <QVBoxLayout>

TopicWidget::TopicWidget()
{
  auto topic_label = new QLabel("Topic Name");
  topic_name = new QLineEdit();
  topic_name->setReadOnly(true);

  auto node_label = new QLabel("Related Nodes");
  node_list = new QTreeWidget();

  node_list->setColumnCount(5);
  node_list->setHeaderLabels({"Node", "Dir", "Type", "Reliability", "Durability"});
  node_list->setRootIsDecorated(false);

  auto layout = new QVBoxLayout();
  layout->addWidget(topic_label);
  layout->addWidget(topic_name);
  layout->addWidget(node_label);
  layout->addWidget(node_list);
  setLayout(layout);
}

void TopicWidget::setGraph(const tkgism_ros_tools::Graph & graph, const QTreeWidget * treewidget, const NodeWidget * nodewidget)
{
  for(const auto & link : graph.links)
  {
    auto item = new QTreeWidgetItem();
    item->setText(0, link->node.c_str());
    item->setText(1, link->is_publish ? "from" : "to");
    item->setText(2, link->type.c_str());
    item->setText(3, link->reliability.c_str());
    item->setText(4, link->durability.c_str());
    item->setData(5, Qt::UserRole, link->topic.c_str());
    node_list->addTopLevelItem(item);
  }
  connect(treewidget, &QTreeWidget::currentItemChanged, this, &TopicWidget::setTopic);
  connect(node_list, &QTreeWidget::currentItemChanged, nodewidget, &NodeWidget::setNode);
}

void TopicWidget::setTopic(QTreeWidgetItem * curr, QTreeWidgetItem *)
{
  topic_name->setText(curr->text(0));
  for (int i = 0; i < node_list->topLevelItemCount(); ++i)
  {
    auto item = node_list->topLevelItem(i);
    item->setHidden(curr->text(0) != item->data(5, Qt::UserRole).toString());
  }
}

NodeWidget::NodeWidget()
{
  auto node_label = new QLabel("Node Name");
  node_name = new QLineEdit();
  node_name->setReadOnly(true);

  auto topic_label = new QLabel("Related Topics");
  topic_list = new QTreeWidget();

  topic_list->setColumnCount(5);
  topic_list->setHeaderLabels({"Topic", "Dir", "Type", "Reliability", "Durability"});
  topic_list->setRootIsDecorated(false);

  auto layout = new QVBoxLayout();
  layout->addWidget(node_label);
  layout->addWidget(node_name);
  layout->addWidget(topic_label);
  layout->addWidget(topic_list);
  setLayout(layout);
}

void NodeWidget::setGraph(const tkgism_ros_tools::Graph & graph, const QTreeWidget * treewidget, const TopicWidget * topicwidget)
{
  for(const auto & link : graph.links)
  {
    auto item = new QTreeWidgetItem();
    item->setText(0, link->topic.c_str());
    item->setText(1, link->is_publish ? "out" : "in");
    item->setText(2, link->type.c_str());
    item->setText(3, link->reliability.c_str());
    item->setText(4, link->durability.c_str());
    item->setData(5, Qt::UserRole, link->node.c_str());
    topic_list->addTopLevelItem(item);
  }
  connect(treewidget, &QTreeWidget::currentItemChanged, this, &NodeWidget::setNode);
  connect(topic_list, &QTreeWidget::currentItemChanged, topicwidget, &TopicWidget::setTopic);
}

void NodeWidget::setNode(QTreeWidgetItem * curr, QTreeWidgetItem *)
{
  node_name->setText(curr->text(0));
  for (int i = 0; i < topic_list->topLevelItemCount(); ++i)
  {
    auto item = topic_list->topLevelItem(i);
    item->setHidden(curr->text(0) != item->data(5, Qt::UserRole).toString());
  }
}

MyWindow::MyWindow(const rclcpp::Node::SharedPtr & node) : QMainWindow(), node_(node)
{
  auto splitter = new QSplitter();
  auto iflist = new QTabWidget();
  auto tlist = new QTreeWidget();
  auto nlist = new QTreeWidget();
  auto tinfo = new TopicWidget();
  auto ninfo = new NodeWidget();

  auto widget = new QWidget();
  auto layout = new QVBoxLayout();
  layout->addWidget(ninfo);
  layout->addWidget(tinfo);
  widget->setLayout(layout);

  iflist->addTab(tlist, "Topic");
  iflist->addTab(nlist, "Node");
  tlist->setColumnCount(2);
  tlist->setHeaderLabels({"Name", "Info"});
  tlist->setRootIsDecorated(false);
  nlist->setColumnCount(2);
  nlist->setHeaderLabels({"Name", "Info"});
  nlist->setRootIsDecorated(false);
  splitter->addWidget(iflist);
  splitter->addWidget(widget);
  setCentralWidget(splitter);

  tkgism_ros_tools::Graph graph;
  graph.update(node_);

  for (const auto & topic : graph.topics)
  {
    auto item = new QTreeWidgetItem();
    item->setText(0, topic.first.c_str());
    tlist->addTopLevelItem(item);
  }
  for (const auto & node : graph.nodes)
  {
    auto item = new QTreeWidgetItem();
    item->setText(0, node.first.c_str());
    nlist->addTopLevelItem(item);
  }

  tinfo->setGraph(graph, tlist, ninfo);
  ninfo->setGraph(graph, nlist, tinfo);
}
