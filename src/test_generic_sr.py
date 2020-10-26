import speech_recognition as sr

r=sr.Recognizer()

while True:
	
	with sr.Microphone() as source:
		print ("Say something:")
		audio=r.listen(source)
	
	try:
		print ("Sphinx text:")
		print (r.recognize_sphinx(audio))
	except:
		print("error reconociendo Sphinx")
		
	try:
		print ("Google Text:")
		print (r.recognize_google(audio, language='es-es'))
	except:
		print("error reconociendo Google")
		
	try:
		print ("Google Text:")
		print (r.recognize_google(audio, language='es-es'))
	except:
		print("error reconociendo Sphinx")
