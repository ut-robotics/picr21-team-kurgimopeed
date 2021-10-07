import atexit
from typing import List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import StreamingResponse, Response
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles

import time
import json
from src.motor_driver import MotorDriver
from src.nuc_led import NucLED

# camera works perfectly, just frames have to finish buffering after reload lol
#from src.RSCamera import RSCamera

led = NucLED()

def default_led():
    led.set_led(led.TYPE_RING, color=led.RING_RED, brightness=0xff, mode=led.MODE_FADE_1HZ)

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

motor_driver = MotorDriver("/dev/ttyACM0")
print("starting motor driver")
motor_driver.start()

def cleanup_motor_driver():
    print("stopping motor driver")
    motor_driver.close()

# TODO: this is never called, because atexit doesnt work on multiprocessed threads.
atexit.register(cleanup_motor_driver)

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

#rs_cam = RSCamera()

@app.get("/depth-feed")
async def depth_feed(request: Request):
    def feed_generator():
        while True:
            yield (b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + rs_cam.depth_frame + b"\r\n\r\n")
            time.sleep(1/25) # big sync fix

    while not rs_cam.is_ready():
        time.sleep(0.2)
    return StreamingResponse(feed_generator(), media_type="multipart/x-mixed-replace;boundary=frame")

@app.get("/color-feed")
async def color_feed(request: Request):
    def feed_generator():
        while True:
            yield (b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + rs_cam.color_frame + b"\r\n\r\n")
            time.sleep(1/25) # big sync fix

    while not rs_cam.is_ready():
        time.sleep(0.2)
    return StreamingResponse(feed_generator(), media_type="multipart/x-mixed-replace;boundary=frame")

@app.get("/")
async def index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})
