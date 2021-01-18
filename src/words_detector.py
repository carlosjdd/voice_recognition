#!/usr/bin/python3

# import the necessary packages
import rospy
import rospkg

import csv

# import the necessary msgs. Example with msg type String_Int_Arrays:
from std_msgs.msg import String
from custom_msgs.msg import String_Int


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
        self.detector_sub = rospy.Subscriber("asr_text", String, self.callback)

        #Define the ROS publishers
        self.detector_pub = rospy.Publisher("asr_word", String_Int, queue_size=0)

        #Define object as msg type
        self.word_detected = String_Int()
        self.word_detected.data_int = 0
        self.word_detected.data_string = [""]

        self.databases()

        print("[INFO] Node started")

    def databases(self):
        rospack = rospkg.RosPack()
        pkg_name = "voice_recognition"			# Name of the ROS package. Is used to take the path of the package
        path_people = rospack.get_path(pkg_name) + "/data/database1_people.csv"
        path_insults = rospack.get_path(pkg_name) + "/data/database2_insults.csv"
        path_name = rospack.get_path(pkg_name) + "/data/database3_robot_name.csv"
        path_police = rospack.get_path(pkg_name) + "/data/database4_policia.csv"
        path_hello = rospack.get_path(pkg_name) + "/data/database5_saludos.csv"
        path_oferta = rospack.get_path(pkg_name) + "/data/database6_ofertas.csv"
        path_cinco = rospack.get_path(pkg_name) + "/data/database7_cinco.csv"

        self.word = [[],[],[],[],[],[],[]]

        with open(path_name) as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=";")	            # Read the csv file
            for row in csv_reader:								        # Go through every row in the csv file
                self.word[0].append(row[0])					            # Save the path of every SVG file into the array

        with open(path_hello) as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=";")	            # Read the csv file
            for row in csv_reader:								        # Go through every row in the csv file
                self.word[1].append(row[0])					            # Save the path of every SVG file into the array

        with open(path_people) as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=";")	            # Read the csv file
            for row in csv_reader:								        # Go through every row in the csv file
                self.word[2].append(row[0])					            # Save the path of every SVG file into the array

        with open(path_insults) as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=";")	            # Read the csv file
            for row in csv_reader:								        # Go through every row in the csv file
                self.word[3].append(row[0])					            # Save the path of every SVG file into the array

        with open(path_police) as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=";")	            # Read the csv file
            for row in csv_reader:								        # Go through every row in the csv file
                self.word[4].append(row[0])					            # Save the path of every SVG file into the array

        with open(path_oferta) as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=";")	            # Read the csv file
            for row in csv_reader:								        # Go through every row in the csv file
                self.word[5].append(row[0])					            # Save the path of every SVG file into the array

        with open(path_cinco) as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=";")	            # Read the csv file
            for row in csv_reader:								        # Go through every row in the csv file
                self.word[6].append(row[0])					            # Save the path of every SVG file into the array


    def detect_word(self, phrase):
        robot_named = False
        for i in range(len(self.word)):
            if robot_named == False:
                for j in self.word[i]:
                    if phrase.find(j) >= 0:
                        self.word_detected.data_int = i
                        self.word_detected.data_string = j
                        self.detector_pub.publish(self.word_detected)
                        if i == 0:
                            robot_named = True
                        print("[DETECTED]:" + j)

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

#        print (data.data)
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
