from launch import LaunchDescription
from launch_ros.actions import Node
from nodeif_launch import Graph


def generate_launch_description():

    # create nodeif_launch graph
    graph = Graph()

    # create nodeif
    talker_node = graph.nodeif(Node, namespace="my_graph", package="nodeif_user1", executable="talker", name="talker_node")
    listener_node = graph.nodeif(Node, namespace="my_graph", package="nodeif_user1", executable="listener", name="listener_node")

    # create connection
    chatter_connection = graph.connection(name="/topic/chatter", type="std_msgs/msg/String")

    # get nodeif socket
    talker_chatter_socket = talker_node.socket("~/message")
    listener_chatter_socket = listener_node.socket("~/message")

    # bind socket to connection
    talker_chatter_socket.bind(chatter_connection)
    listener_chatter_socket.bind(chatter_connection)

    # create launch description
    return LaunchDescription(graph.description())
