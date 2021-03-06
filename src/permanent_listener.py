#!/usr/bin/python3

# import the necessary packages
import rospy
import time

# import the necessary msgs. Example with msg type String_Int_Arrays:
from std_msgs.msg import Float32
from std_msgs.msg import Bool

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
        self.stop_sub = rospy.Subscriber("stop_permanent_asr", Bool, self.cb_stop)

        #Define the ROS publishers
        self.listener1_pub = rospy.Publisher("recognize_voice1", Float32, queue_size=0)
        self.listener2_pub = rospy.Publisher("recognize_voice2", Float32, queue_size=0)
        self.listener3_pub = rospy.Publisher("recognize_voice3", Float32, queue_size=0)
        self.listener4_pub = rospy.Publisher("recognize_voice4", Float32, queue_size=0)

        #Define whether it is listening permanently or not.
        self.stop_asr = False

        #Define the time in seconds to listen every phrase
        self.duration = 2.0

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
            if self.stop_asr == False:
                self.listener1_pub.publish(self.listen)
                time.sleep(self.duration*0.8)
                self.listener2_pub.publish(self.listen)
                time.sleep(self.duration*0.8)
                self.listener3_pub.publish(self.listen)
                time.sleep(self.duration*0.8)
                self.listener4_pub.publish(self.listen)
            time.sleep(self.duration*0.8)


    def stopping_node(self):
        """ROS closing node

        Is the function called when ROS node is closed."""
        print("\n\nBye bye! :)\n\n")

    def callback(self, data):
        """ROS callback

        This void is executed when a message is received"""
        self.duration = data.data
        self.listen.data = self.duration

    def cb_stop(self, data):
        """ROS callback to stop permanently listening
        """
        self.stop_asr=data.data


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
