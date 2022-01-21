import abc
import rospy

"""
    base class for all ALEX ROS nodes
"""
class ALEXNode(abc, metaclass=abc.ABCMeta):

    # one callback per node?

    """
        initialize ROS node

        name: name of the ROS (ALEX) node
    """
    def __init__(self, name):
        self.name  = name # name can possibly be used for logging later
        publishers = {}

        rospy.init_node(name)

    """
        subscribe to a topic

        topic:    the topic to subscribe to
        callback: the callback to subscribe with
    """
    def subscribe(self, topic, callback):
        rospy.Subscriber(topic, String, callback)

    """
        create a publisher

        topic: the topic to which the new publisher will publish
    """
    def createPublisher(self, topic, type = String):
        if not topic in publishers
            publishers[topic] = rospy.Publisher(topic, type)

    """
        publish a message

        topic:   the topic to publish to
        message: the message to be published
    """
    def publish(self, topic, message):
        if topic in publishers:
            publishers[topic].publish(message)

    def log(self):
        pass

    """
        get the status of the ALEXNode

        to be defined by child classes
    """
    @abstractmethod
    def status(self):
        raise NotImplementedError
