import atexit
from typing import List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import StreamingResponse, Response, JSONResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles

import time
import json

from src.motor_driver import MotorDriver
from src.ImageProcess import ImageProcess
from src.nuc_led import NucLED

# camera works perfectly, just frames have to finish buffering after reload lol

led = NucLED()

def default_led():
    led.set_led(led.TYPE_RING, color=led.RING_RED, brightness=64, mode=led.MODE_FADE_1HZ)

default_led()

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

templates = Jinja2Templates(directory="templates")

class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            await connection.send_text(message)

    async def broadcast_json(self, data):
        for connection in self.active_connections:
            await connection.send_json(data)

    async def motor_driver_broadcast(self, value):
        print(value)
        await self.broadcast_json(json.dumps(value))

manager = ConnectionManager()

motor_driver = MotorDriver()
motor_driver.start()

image_proccess = ImageProcess(motor_driver)
image_proccess.start()

@app.on_event("shutdown")
async def shutdown_event():
    motor_driver.close()
    image_proccess.stop()
    led.set_led(led.TYPE_RING, 64, led.MODE_FADE_05HZ, led.RING_PINK)

@app.websocket("/ws/{client_id}")
async def websocket_endpoint(websocket: WebSocket, client_id: int):
    await manager.connect(websocket)
    try:
        while True:
            led.set_led(led.TYPE_RING, brightness=0xff, mode=led.MODE_ON)
            text = await websocket.receive_text()
            await manager.send_personal_message(f"You wrote: {text}", websocket)
            data = json.loads(text)
            print(data)
            speed, direction, turn, thrower, enable = [data[i] for i in ["speed", "direction", "turn", "thrower", "enable"]]
            if enable:
                led.set_led(led.TYPE_RING, color=led.RING_GREEN)
                motor_driver.send(speed=speed, direction=direction, turn_speed=turn, thrower=thrower, callback=None)
            else:
                led.set_led(led.TYPE_RING, color=led.RING_RED)
                motor_driver.stop()
    except WebSocketDisconnect:
        default_led()
        motor_driver.stop()
        manager.disconnect(websocket)
        await manager.broadcast(f"Client #{client_id} left the chat")

@app.get("/depth-feed")
def depth_feed(request: Request):
    def feed_generator():
        while not image_proccess.stop:
            yield (b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + image_proccess.get_frame1() + b"\r\n\r\n")
            time.sleep(1/25) # big sync fix

    while not image_proccess.has_new_frame1():
        time.sleep(0.2)
    return StreamingResponse(feed_generator(), status_code=206, media_type="multipart/x-mixed-replace;boundary=frame")


@app.get("/color-feed")
def color_feed(request: Request):
    def feed_generator():
        while not image_proccess.stop:
            yield (b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + image_proccess.get_frame2() + b"\r\n\r\n")
            time.sleep(1/25) # big sync fix

    while not image_proccess.has_new_frame2():
        time.sleep(0.2) 
    return StreamingResponse(feed_generator(), status_code=206, media_type="multipart/x-mixed-replace;boundary=frame")

@app.get("/")
async def index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/config")
def load_config(request: Request):
    with open("../config/threshold_config.json", "r") as f:
        return JSONResponse(content=json.load(f))

thrower_note_lut = {
    "f6": 246,
    "g6": 275,
    "a6": 303,
    "c7": 357,
    "e7": 443,
    "f7": 466,
    "a6b": 288,
    "a7": 581,
    "a7b": 551,
    "g7": 520,
    "g7b": 494,
    "e7b": 418,
    "b6b": 321,
    "d7": 397,
    "d7b": 376,
    "b6": 338,
    "0": 0
}

imp_march = [
("a6", 1),
("a6", 1),
("a6", 1),
("f6", 3/4),
("c7", 1/4),

("a6", 1),
("f6", 3/4),
("c7", 1/4),
("a6", 2),

("e7", 1),
("e7", 1),
("e7", 1),
("f7", 3/4),
("c7", 1/4),

("a6b", 1),
("f6", 3/4),
("c7", 1/4),
("a6", 2),

("a7", 1),
("a6", 3/4),
("a6", 1/4),
("a7", 1),
("a7b", 3/4),
("g7", 1/4),

("g7b", 1/4),
("f7", 1/4),
("g7b", 1/2),
("0", 1/2),
("b6b", 1/2),
("e7b", 1),
("d7", 3/4),
("d7b", 1/4),

("c7", 1/4),
("b6", 1/4),
("c7", 1/2),
("0", 1/2),
("f6", 1/2),
("a6b", 1),
("f6", 3/4),
("a6", 1/4),

("c7", 1),
("a6", 3/4),
("c7", 1/4),
("e7", 2),

("a7", 1),
("a6", 3/4),
("a6", 1/4),
("a7", 1),
("a7b", 1/2),
("g7", 1/2),

("g7b", 1/4),
("f7", 1/4),
("g7b", 1/2),
("0", 1/2),
("b6b", 1/2),
("e7b", 1),
("d7", 3/4),
("d7b", 1/4),

("c7", 1/4),
("b6", 1/4),
("c7", 1/2),
("0", 1/2),
("f6", 1/2),
("a6b", 1),
("f6", 3/4),
("c7", 1/4),

("a6", 1),
("f6", 3/4),
("c7", 1/4),
("a6", 1)
]

@app.post("/play_march")
async def play_march(request: Request):
    #j = await request.json()
    bpm = 103
    for note, dur in imp_march:
        motor_driver.send(thrower=thrower_note_lut[note])
        time.sleep(dur * (60 / bpm) * 0.9)
        motor_driver.send(thrower=0)
        time.sleep(dur * (60 / bpm) * 0.1)

@app.post("/config")
async def save_config(request: Request):
    j = await request.json()
    image_proccess.threshold_values = list(j.values())
    with open("../config/threshold_config.json", "w") as f:
        json.dump(j, f)
