from vosk import Model, KaldiRecognizer
import pyaudio
import json
import pyttsx3
import time


#load model and initialize
model = Model("/Users/ray/Downloads/vosk-model-small-en-us-0.15")
recognizer = KaldiRecognizer(model, 16000)
cap = pyaudio.PyAudio()
stream = cap.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)
stream.start_stream()
engine = pyttsx3.init()


#recognize
while True:
    data = stream.read(4096, exception_on_overflow=False)
    if recognizer.AcceptWaveform(data):
        result = recognizer.Result()
        text = json.loads(result)
        text = text.get("text")
        print(text)
        if text != "":
            #time.sleep(500)
            engine.say(text)
            engine.runAndWait()
        else:
            print("waiting")
