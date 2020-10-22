import speech_recognition as sr

audio = "Prueba.wav"

re = sr.Recognizer()

with sr.AudioFile(audio) as source:
        info_audio=re.record(source)
        texto=re.recognize_google(info_audio)
        print("El texto reconocido es:\n")
        print(texto)
        print("\nSaliendo\n")
