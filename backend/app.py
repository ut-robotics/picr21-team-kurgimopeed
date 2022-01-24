from typing import List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request, Form
from fastapi.responses import StreamingResponse, Response, JSONResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles

import time
import json
import cv2

#from src.MotorDriver import MotorDriver
#from src.ImageProcess import ImageProcess
#from src.DrivingLogic import DrivingLogic
from src.GoalDetector import GoalDetector
from src.nuc_led import NucLED
from src.MusicBox import MusicBox
from src.RobotController import RobotController

# camera works perfectly, just frames have to finish buffering after reload lol

led = NucLED()

def default_led():
    #led.set_led(led.TYPE_RING, color=led.RING_RED, brightness=64, mode=led.MODE_FADE_1HZ)
    led.set_led(led.TYPE_RING, color=led.RING_RED, brightness=64, mode=led.MODE_OFF)

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

#motor_driver = MotorDriver()
#motor_driver.start()

#driving_logic = DrivingLogic(motor_driver)
#image_proccess = ImageProcess(driving_logic)
#image_proccess.start()

robot_controller = RobotController()
robot_controller.start()

musicbox = MusicBox(robot_controller.motor_driver)

@app.on_event("shutdown")
async def shutdown_event():
    robot_controller.close()
    #motor_driver.close()
    #image_proccess.stop()
    led.set_led(led.TYPE_RING, 64, led.MODE_FADE_05HZ, led.RING_PINK)

@app.websocket("/ws/{client_id}")
async def websocket_endpoint(websocket: WebSocket, client_id: int):
    await manager.connect(websocket)
    try:
        while True:
            #led.set_led(led.TYPE_RING, brightness=0xff, mode=led.MODE_ON)
            text = await websocket.receive_text()
            await manager.send_personal_message(f"You wrote: {text}", websocket)
            data = json.loads(text)
            #print(data)
            if "pid" in data:
                data = data["pid"]
                #driving_logic.circle_turn_pid.tunings = data[:3]
                #driving_logic.circle_turn_pid.setpoint = data[-1]
            else:
                speed, direction, turn, thrower, ball_hold, thrower_angle, enable, driving_enable = [data[i] for i in ["speed", "direction", "turn", "thrower", "ball_hold", "thrower_angle", "enable", "drive_enable"]]
                if enable:
                    robot_controller.driving_logic.stop()
                    robot_controller.motor_driver.send(speed=speed, direction=direction, turn_speed=turn, thrower=thrower, servo_angle=thrower_angle, servo_hold=ball_hold)
                elif driving_enable:
                    robot_controller.driving_logic.start()
                else:
                    time.sleep(0.1)
                    #led.set_led(led.TYPE_RING, color=led.RING_RED)
                    if not musicbox.playing:
                        robot_controller.motor_driver.stop()
                    robot_controller.driving_logic.stop()


    except WebSocketDisconnect:
        default_led()
        robot_controller.motor_driver.stop()
        manager.disconnect(websocket)
        await manager.broadcast(f"Client #{client_id} left the chat")

@app.get("/depth-feed")
def depth_feed(request: Request):
    def feed_generator():
        while not robot_controller.stop:
            f =  cv2.imencode(".jpg", robot_controller.image_proccess.debug_frame1)[1].tobytes()
            yield (b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + f + b"\r\n\r\n")
            time.sleep(1/10) # big sync fix

    while not robot_controller.image_proccess.has_new_frame1():
        time.sleep(0.2)
    return StreamingResponse(feed_generator(), status_code=206, media_type="multipart/x-mixed-replace;boundary=frame")


@app.get("/color-feed")
def color_feed(request: Request):
    def feed_generator():
        while not robot_controller.stop:
            f =  cv2.imencode(".jpg", robot_controller.image_proccess.debug_frame2)[1].tobytes()
            yield (b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + f + b"\r\n\r\n")
            time.sleep(1/10) # big sync fix

    while not robot_controller.image_proccess.has_new_frame2():
        time.sleep(0.2) 
    return StreamingResponse(feed_generator(), status_code=206, media_type="multipart/x-mixed-replace;boundary=frame")

@app.get("/")
async def index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/trackbar-config")
async def load_config(request: Request):
    with open(robot_controller.image_proccess.trackbar_path, "r") as f:
        return JSONResponse(content=json.load(f))

@app.post("/playmusic")
async def play_march(song: str = Form(...)):
    musicbox.play(song)

@app.post("/stopmusic")
async def play_march(request: Request):
    musicbox.stop()

@app.post("/calibrate-camera")
async def play_march(request: Request):
    status = 200 if robot_controller.image_proccess.calibrate() else 417 #ok vs ecpectation failed
    return Response(status_code=status)

@app.post("/set_debug_color_mask")
async def play_march(request: Request):
    j = await request.json()
    robot_controller.image_proccess.show_mask = False if j["state"] == "OFF" else True

@app.post("/trackbar-config")
async def save_config(request: Request):
    j = await request.json()
    #print(j)
    robot_controller.image_proccess.threshold_values[j["threshold_config"]] = j["threshold_values"]
    robot_controller.image_proccess.active_threshold_config = j["threshold_config"]
    with open(robot_controller.image_proccess.trackbar_path, "w") as f:
        json.dump(robot_controller.image_proccess.threshold_values, f)

#@app.get("/get-target-goal")
#async def get_target_goal(request: Request):
#    return JSONResponse(content)
#    j = await request.json()
#    driving_logic.target_goal = GoalDetector.ID_BLUE if j["target_goal"] == "blue_goal" else GoalDetector.ID_PINK

@app.post("/set-target-goal")
async def set_target_goal(request: Request):
    j = await request.json()
    robot_controller.driving_logic.target_goal = GoalDetector.ID_BLUE if j["target_goal"] == "blue_goal" else GoalDetector.ID_PINK

@app.get("/court")
async def court(request: Request):
    return templates.TemplateResponse("court.html", {"request": request})