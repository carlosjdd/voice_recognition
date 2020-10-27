#!/usr/bin/python3

# import the necessary packages
import rospy
import time

# import the necessary msgs. Example with msg type String_Int_Arrays:
from std_msgs.msg import Float32

class listener():
    """ Class listener.

    This class sends all the time messages to listen and recognize voice permanently.
    """

    def __init__(self):
        """Class constructor

        It is the constructor of the class. It does:
        - Subscribe to topic to modify the time to listen every phrase
        - Define the publisher to ask the recognizer to recognize voice
        """

        #Subscribe to ROS topics
        self.listener_sub = rospy.Subscriber("listening_duration", Float32, self.callback)

        #Define the ROS publishers
        self.listener_pub = rospy.Publisher("recognize_voice", Float32, queue_size=0)

        #Define the time in seconds to listen every phrase
        self.duration = 3.0

        #Define object as msg type
        self.listen = Float32()
        self.listen.data = self.duration

        print("[INFO] Node started")


    def run_loop(self):
        """ Infinite loop.

        When ROS is closed, it exits.
        It sends a message every "self.duration" time
        """
        while not rospy.is_shutdown():
            #functions to repeat until the node is closed
            self.listener_pub.publish(self.listen)
            time.sleep(self.duration)

    def stopping_node(self):
        """ROS closing node

        Is the function called when ROS node is closed."""
        print("\n\nBye bye! :)\n\n")

    def callback(self, data):
        """ROS callback

        This void is executed when a message is received"""
        self.duration = data.data
        self.listen.data = self.duration


if __name__=='__main__':
    """ Main void.

    Is the main void executed when started. It does:
    - Start the node
    - Create an object of the class
    - Run the node

    """
    try:
        rospy.init_node('listener_node')       # Init ROS node

        list_object = listener()
        rospy.on_shutdown(list_object.stopping_node)   #When ROS is closed, this void is executed

        list_object.run_loop()

    except rospy.ROSInterruptException:
        pass
