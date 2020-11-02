#!/usr/bin/python

# import the necessary packages
import rospy
from wit import Wit

from unidecode import unidecode
import sounddevice as sd
from scipy.io.wavfile import write

# import the necessary msgs. Example with msg type String_Int_Arrays:
from std_msgs.msg import Float32
from std_msgs.msg import String

class voice_recognitor4():
    """ Class voice_recognitor

    This class allows to recognite the voice during an indicated time in seconds.
    """

    def __init__(self):
        """Class constructor

        It is the constructor of the class. It does:
        -Subscribe to recognize_voice topic
        -Publish the asr text
        """

        #Subscribe to ROS topics
        self.asr_sub = rospy.Subscriber("recognize_voice4", Float32, self.callback)

        #Define the ROS publishers
        self.asr_pub = rospy.Publisher("asr_text", String, queue_size=0)

        #Define object as msg type
        self.asr_msg = String()
        self.asr_msg.data = ""

        self.duration=3.0

        self.configuration()

        print("[INFO] Node started")

    def configuration(self):
        """Configuration void.

        In this void the token of the wit.ai client is defined.
        And it is set the sample_rate of the text recorded.
        """
        token="VW6CLYS2BCPOCWSATWXNZNVTLSEH3WJM"
        self.client = Wit(token)

        self.sample_rate=44100


    def recognize(self,duration):
        """Void to recognize voice

        First, the voice is recorded with the duration time set.
        After that, the text is recognized and published.
        """
        myrecording = sd.rec(int(duration*self.sample_rate), samplerate=self.sample_rate, channels=2)
        sd.wait()
        write('output4.wav', self.sample_rate, myrecording)

        try:
            with open('output4.wav', 'rf') as f:
                answ=self.client.speech(f, {'Content-Type': 'audio/wav'})
            text = unidecode(answ[u'text'])
        except:
            text=""
        print(text)
        self.asr_msg.data = text
        #Publish msg
        self.asr_pub.publish(self.asr_msg)

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

        This void is executed when a message is received.
        It simply calls the function to recognize giving the duration of the recording"""
        self.recognize(data.data)



if __name__=='__main__':
    """ Main void.

    Is the main void executed when started. It does:
    - Start the node
    - Create an object of the class
    - Run the node

    """
    try:
        rospy.init_node('asr_node4')       # Init ROS node

        asr_object = voice_recognitor4()
        rospy.on_shutdown(asr_object.stopping_node)   #When ROS is closed, this void is executed

        asr_object.run_loop()

    except rospy.ROSInterruptException:
        pass
