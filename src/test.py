import speech_recognition as sr

audio = "Prueba2.wav"
audio2 = "Test_en.wav"

re = sr.Recognizer()

with sr.AudioFile(audio) as source:
    info_audio=re.record(source)
    texto=re.recognize_google(info_audio, language="es-ES")
    print("El texto reconocido es:\n")
    print(texto)
    print("\nAhora en Ingles:\n")

with sr.AudioFile(audio2) as source:
    info_audio=re.record(source)
    texto=re.recognize_google(info_audio)
    print("El texto reconocido es:\n")
    print(texto)
    print("\nSaliendo:\n")
