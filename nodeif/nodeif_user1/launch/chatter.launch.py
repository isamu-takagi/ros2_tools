import nodeif_launch as Launch_graph
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_graph():

    # create Launch graph
    graph = Launch_graph.create_graph()

    # create node process
    talker = graph.create_process(Node, namespace="my_graph", package="nodeif_user1", executable="talker", name="talker_node")
    listener = graph.create_process(Node, namespace="my_graph", package="nodeif_user1", executable="listener", name="listener_node")

    # create topic channel
    message = graph.create_channel(name="/topic/chatter", type="std_msgs/msg/String")

    # set up connection
    message.bind([
        (talker, "~/message"),
        (listener, "~/message"),
    ])

    return graph


def generate_launch_description():

    # convert to launch description
    return LaunchDescription(generate_launch_graph().description())
