#!/usr/bin/python3

# import the necessary packages
import rospy
import rospkg

import csv

# import the necessary msgs. Example with msg type String_Int_Arrays:
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray


class detector():
    """ Class class_name.

    Info about the class
    """

    def __init__(self):
        """Class constructor

        It is the constructor of the class. It does:
        - Subscribe to asr topic
        - Define publisher for asr detected word
        - Create msg type to send
        - take path to every database
        """

        #Subscribe to ROS topics
        self.detector_sub = rospy.Subscriber("asr_full_text", String, self.callback)

        #Define the ROS publishers
        self.fragance_pub = rospy.Publisher("fragances", UInt8MultiArray, queue_size=0)

        #Define object as msg type
        self.fragance_msg = UInt8MultiArray()
        self.fragance_msg.data = []

        self.databases()

        print("[INFO] Node started")

    def databases(self):
        rospack = rospkg.RosPack()
        pkg_name = "voice_recognition"			# Name of the ROS package. Is used to take the path of the package
        path_fragances = rospack.get_path(pkg_name) + "/data/fragances.csv"

        self.word = []
        self.frag1 = []
        self.frag2 = []

        with open(path_fragances) as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=";")	            # Read the csv file
            for row in csv_reader:								        # Go through every row in the csv file
                self.word.append(row[0])					            # Save the path of every SVG file into the array
                self.frag1.append(row[1])
                self.frag2.append(row[2])

    def detect_word(self, phrase):

        self.fragance_msg.data = []
        check = False

        for j in range(len(self.word)):
            if phrase.find(self.word[j]) >= 0:
                if int(self.frag1[j]) in self.fragance_msg.data:
                    pass
                else:
                    self.fragance_msg.data.append(int(self.frag1[j]))
                if int(self.frag2[j]) != 0:
                    if int(self.frag2[j]) in self.fragance_msg.data:
                        pass
                    else:
                        self.fragance_msg.data.append(int(self.frag2[j]))
                print (self.word[j])

        self.fragance_pub.publish(self.fragance_msg)

    def run_loop(self):
        """ Infinite loop.

        When ROS is closed, it exits.
        """
        while not rospy.is_shutdown():
            #functions to repeat until the node is closed
            rospy.spin()

    def stopping_node(self):
        """ROS closing node

        Is the function called when ROS node is closed."""
        print("\n\nBye bye! :)\n\n")

    def callback(self, data):
        """ROS callback

        This void is executed when a message is received"""
        self.detect_word(data.data)


if __name__=='__main__':
    """ Main void.

    Is the main void executed when started. It does:
    - Start the node
    - Create an object of the class
    - Run the node

    """
    try:
        rospy.init_node('detector_node')       # Init ROS node

        word_detector = detector()
        rospy.on_shutdown(word_detector.stopping_node)   #When ROS is closed, this void is executed

        word_detector.run_loop()

    except rospy.ROSInterruptException:
        pass
