import speech_recognition as sr

r=sr.Recognizer()
with sr.Microphone() as source:
	print ("Say something:")
	audio=r.listen(source)
	
try:
	print ("Text recognized:")
	print (r.recognize_sphinx(audio))
except:
	print("error reconociendo")
