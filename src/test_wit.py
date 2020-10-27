from wit import Wit
from unidecode import unidecode
from time import time
#import speech_recognition as sr

import sounddevice as sd
from scipy.io.wavfile import write

fs=44100 #sample rate
seconds=3


token="VW6CLYS2BCPOCWSATWXNZNVTLSEH3WJM"

client = Wit(token)

while True:
	print ("recording")
	start_time=time()

	myrecording = sd.rec(int(seconds*fs), samplerate=fs, channels=2)
	sd.wait()
	rec_time=time() - start_time
	write('output.wav', fs, myrecording)

	#r=sr.Recognizer()
	#with sr.Microphone() as source:
	#	print ("Say something:")
	#	audio=r.listen(source)

	print ("\nVoice recognition")
	start_time=time()

	try:
		with open('output.wav', 'rf') as f:
			answ=client.speech(f, {'Content-Type': 'audio/wav'})
		texto = unidecode(answ[u'text'])
	except:
		texto=""

	det_time=time()-start_time

	print(texto)

	if (texto.find("cabron")>=0):
		print ("Que te jodan\n")

	print (rec_time)
	print (det_time)

	#answ=client.speech(audio)
	#print(str(answ))


	#print ("\nModo interactivo")
	#client.interactive()

