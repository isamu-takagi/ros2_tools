#include <QMainWindow>
#include <QSplitter>
#include <QTreeWidget>
#include <QTabWidget>
#include <QLabel>
#include <QLineEdit>
#include "rclcpp/rclcpp.hpp"
#include "graph.hpp"

class TopicWidget;
class NodeWidget;

class TopicWidget : public QWidget
{
  Q_OBJECT

  public:
    TopicWidget();
    void setGraph(const tkgism_ros_tools::Graph & graph, const QTreeWidget * treewidget, const NodeWidget * nodewidget);

  public slots:
    void setTopic(QTreeWidgetItem * curr, QTreeWidgetItem * prev);

  private:
    QLineEdit * topic_name;
    QTreeWidget * node_list;
};

class NodeWidget : public QWidget
{
  Q_OBJECT

  public:
    NodeWidget();
    void setGraph(const tkgism_ros_tools::Graph & graph, const QTreeWidget * treewidget, const TopicWidget * topicwidget);

  public slots:
    void setNode(QTreeWidgetItem * curr, QTreeWidgetItem * prev);

  private:
    QLineEdit * node_name;
    QTreeWidget * topic_list;
};

class MyWindow : public QMainWindow
{
  Q_OBJECT

  public:
    MyWindow(const rclcpp::Node::SharedPtr & node);

  private:
    rclcpp::Node::SharedPtr node_;
};
