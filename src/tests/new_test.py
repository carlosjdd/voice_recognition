import speech_recognition as sr
import time

r = sr.Recognizer()

mic = sr.Microphone(device_index=0)
#print(sr.Microphone.list_microphone_names())
while True:
	#audio_file = sr.AudioFile('/home/pi/catkin_ws/src/voice_recognition/src/tests/harvard.wav')
	with mic as source:
		#audio = r.record(source, duration=4)
		#audio = r.adjust_for_ambient_noise(source)
		start=time.time()
		print('starting')
	
		audio = r.listen(source, phrase_time_limit=5)
		listened = time.time()
		print (listened-start)

		output = r.recognize_google(audio, language="es-ES")
		finished = time.time()
		print (finished-listened)
	
		##output = r.recognize_bing(audio, language="es-ES")
		##output = r.recognize_google_cloud(audio, language="es-ES")
		##output = r.recognize_houndify(audio)
		##output = r.recognize_ibm(audio, language="es-ES", username="carlycharly@hotmail.com", password="3pj58q.L")
		##output = r.recognize_sphinx(audio, language="es-ES")
		#output = r.recognize_wit(audio, key="VW6CLYS2BCPOCWSATWXNZNVTLSEH3WJM")

		print (output)
