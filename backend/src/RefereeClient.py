import websockets
import asyncio
import json
from threading import Thread
from src.GoalDetector import GoalDetector 

class RefereeClient():
    def __init__(self):
        self.ip = "192.168.3.38"
        self.id = "kurk"
        self.port = 8111

        self.uri = f"ws://{self.ip}:{self.port}"

        self.start_func = None
        self.stop_func = None

        self.stop = False

        self.thread = Thread(target=self.stub, daemon=True)

    def on_start_event(self, fun):
        self.start_func = fun

    def on_stop_event(self, fun):
        self.stop_func = fun

    def stub(self):
        loop = asyncio.new_event_loop()
        loop.run_until_complete(self.ws_client())
        loop.run_forever()

    async def ws_client(self):
        print("Referee ws started")
        while not self.stop:
            try:
                async with websockets.connect(self.uri) as ws:
                    while not self.stop:
                        msg = await ws.recv()
                        j = json.loads(msg)
                        #print(j)
                        try:
                            if j["signal"] == "start":
                                if not self.id in j["targets"]:
                                    continue

                                index = j["targets"].index(self.id)
                                goal = GoalDetector.ID_BLUE if j["baskets"][index] == "blue" else GoalDetector.ID_PINK
                                self.start_func(goal)
                            elif j["signal"] == "stop":
                                if self.id in j["targets"]:
                                    self.stop_func()
                        except Exception as e:
                            print(e)
            except websockets.exceptions.ConnectionClosedError:
                pass



    def start(self):
        self.thread.start()

    def close(self):
        self.stop = True