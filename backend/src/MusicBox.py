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
        self.STOP = True

        self.tone_length = 0.9 #in % how much tone should last and other for stop 

        self.player = None

    def play(self, song):
        self.stop()
        self.player = Thread(target=self.run, args=(song,))
        self.player.daemon = True
        self.player.start()

    def stop(self):
        self.STOP = True
        if self.player is not None:
            self.player.join()
            self.player = None

    def run(self, song):
        sheet = SimpleQueue()
        bpm = None
        with open(self.path+song) as f:
            for line in f.read().split("\n"):
                if line != "":
                    if bpm is None:
                        bpm = int(line)
                    else:
                        sheet.put(line.split(" "))

        self.STOP = False
        
        while not sheet.empty() and not self.STOP:
            note, length = sheet.get()
            length = eval(length)*(60 / bpm)

            self.motor_driver.send(thrower=self.get_speed(note))

            start = time.time()
            while time.time()-start < length*self.tone_length:
                if self.STOP:
                    break

            self.motor_driver.send(thrower=0)

            while time.time()-start < length:
                if self.STOP:
                    break

        self.motor_driver.send(thrower=0)

if __name__ == "__main__":
    music = MusicBox(None)
    music.play("imperial_march")
    music.join()





