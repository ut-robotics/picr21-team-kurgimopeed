from queue import SimpleQueue
import time
from threading import Thread

import os, json

class ToneMapper():
    def __init__(self):
        self.notes = {"c":0.5, "d":1.5, "e":2.5, "f":3, "g":4, "a":5, "b":6}
        self.modulations = {"b":-0.5, "#":0.5}

        self.path = "../config/music_lib/"

        self.speed_function_consts = {}
        if os.path.exists(self.path+"function_const.json"):
            with open(self.path+"function_const.json") as f:
                self.speed_function_consts = json.load(f)
        else:
            a, b = self.fit_model()
            self.speed_function_consts = {"a":a, "b":b}
            with open(self.path+"function_const.json", "w") as f:
                json.dump(self.speed_function_consts, f)

    def speed_function(self, x, a, b):
            return a*x**b

    def fit_model(self):
        import numpy as np
        import scipy.optimize as opt
        import matplotlib.pyplot as plt

        X = []
        Y = []
        with open(self.path+"tone.map") as f:
            for line in f.read().split("\n"):
                if line != "":
                    note, speed = line.split(" ")
                    X.append(self.note2number(note))
                    Y.append(int(speed))

        optimized_parameters, _ = opt.curve_fit(
            self.speed_function,
            X,
            Y,
            bounds=([-np.inf, -np.inf], [np.inf, np.inf])
            )

        # Calculate points with the optimized parameters
        x_data_fit = np.linspace(min(X), max(X), 100)
        y_data_fit = self.speed_function(x_data_fit, *optimized_parameters)

        # Plot the data
        plt.plot(X, Y, ".", label="measured data")
        plt.plot(x_data_fit, y_data_fit, label="fitted data")

        # Show the graph
        plt.legend()
        plt.xlabel("Blob size (px)")
        plt.ylabel("Distance (mm)")
        #plt.show()

        return optimized_parameters

    def note2number(self, s):
        if s == "0":
            return 0

        note, octave, *mod = [char for char in s]
        nr = self.notes[note]+1+(int(octave)-1)*6
        if len(mod):
            nr += self.modulations[mod[0]]

        return nr
        

    def get_speed(self, note):
        a, b = self.speed_function_consts["a"], self.speed_function_consts["b"]
        speed = self.speed_function(self.note2number(note), a, b)
        if speed < 0:
            return 0
        return int(speed)


class MusicBox(ToneMapper):
    def __init__(self, motor_driver):
        super().__init__()
        self.motor_driver = motor_driver
        self.stop = True

        self.tone_length = 0.9 #in % how much tone should last and other for stop 

        self.player = Thread(target=self.run)
        self.player.daemon = True

        self.song = None

    def play(self, song):
        if self.player.is_alive():
            self.stop()
            self.join()
        self.song = song
        self.player.start()

    def stop(self):
        self.stop = True
    
    def join(self):
        self.player.join()

    def run(self):
        sheet = SimpleQueue()
        bpm = None
        with open(self.path+self.song) as f:
            for line in f.read().split("\n"):
                if line != "":
                    if bpm is None:
                        bpm = int(line)
                    else:
                        sheet.put(line.split(" "))

        self.stop = False
        
        while not sheet.empty() or not self.stop:
            note, length = sheet.get()
            length = eval(length)*(60 / bpm)

            self.motor_driver.send(thrower=self.get_speed(note))

            start = time.time()
            while time.time()-start >= length*self.tone_length:
                if self.stop:
                    break

            self.motor_driver.send(thrower=0)

            start = time.time()
            while time.time()-start >= length*(1 - self.tone_length):
                if self.stop:
                    break

if __name__ == "__main__":
    music = MusicBox(None)
    music.play("imperial_march")
    music.join()





