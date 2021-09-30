from flask import Flask, render_template, request, Response
import time, asyncio
import websockets

from RSCamera import RSCamera

app = Flask(__name__)

rs_cam = None

@app.route("/depth-feed")
def depth_feed():
    def feed_generator():
        while True:
            yield (b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + rs_cam.depth_frame + b"\r\n\r\n")
            time.sleep(1/25) # big sync fix

    while not rs_cam.is_ready():
        time.sleep(0.2)
    return Response(feed_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/color-feed")
def color_feed():
    def feed_generator():
        while True:
            yield (b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + rs_cam.color_frame + b"\r\n\r\n")
            time.sleep(1/25) # big sync fix

    while not rs_cam.is_ready():
        time.sleep(0.2)
    return Response(feed_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/")
def index():
    return render_template("index.html")

from flask_sock import Sock
sock = Sock(app)

@sock.route("/echo")
def echo(ws):
    while True:
        data = ws.receive()
        ws.send(data)

@app.before_first_request
def setup():
    global rs_cam

    # initialize camera and start frame processing
    rs_cam = RSCamera()

if __name__ == "__main__":
    app.run( port=5000)
