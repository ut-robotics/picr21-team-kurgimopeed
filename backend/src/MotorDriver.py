from serial import Serial
from serial.tools.list_ports import comports
from threading import Thread
from queue import Queue
from math import pi, sin
import time

from src.MainBoardComm import MainBoardComm
from src.Tools import linear_map

STOP_CODE = 0 #random defined number of STOP_CODE to be treated as special number

class MotorControllerHandler():
    def __init__(self, port, baudrate, timeout, queue):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.motor1 = 0
        self.motor2 = 0
        self.motor3 = 0
        self.thrower = 0
        self.servo_angle = 0
        self.servo_hold = 0
        self.disable_failsafe = 0

        self.thread = Thread(target=self.run, args=(queue,), daemon=True)

        self.MAX_SPEED = 65535

    def start(self, queue):
        self.thread.start()
        print("Motor controller started")

    def set_motors(self, data):
        motor1, motor2, motor3, thrower, servo_angle, servo_hold = data
        if motor1 is not None:
            self.motor1 = motor1
        if motor2 is not None:
            self.motor2 = motor2
        if motor3 is not None:
            self.motor3 = motor3
        if thrower is not None:
            self.thrower = thrower
        if servo_angle is not None:
            self.servo_angle = servo_angle
        if servo_hold is not None:
            self.servo_hold = servo_hold
            
    def run(self, queue):
        with Serial(self.port, self.baudrate, timeout=self.timeout) as ser:
            comm = MainBoardComm(ser)
            reset_mainboard = True
            while True:
                callback = None #by default there is no callback
                if not queue.empty():
                    data = queue.get() #get first item from queue
                    if data == STOP_CODE:
                        break
                    *motors, callback = data #extract callback from queue data and assume that everything else is motor data
                    self.set_motors(motors) #sets local motor speeds if changed

                command = 0
                if self.motor1 < 0:
                    command |= 0b1
                if self.motor2 < 0:
                    command |= 0b10
                if self.motor3 < 0:
                    command |= 0b100
                if reset_mainboard:
                    command |= 0b1000
                send_error = comm.send_data(abs(self.motor1), abs(self.motor2), abs(self.motor3), 
                                            self.thrower, self.servo_angle, self.servo_hold, command)
                recv = comm.receive_data()

                if callback is not None and recv is not None:
                    #print("received data from mainborad:", recv)
                    callback(recv)
                    pass
                reset_mainboard = False
                time.sleep(0.02) #limit to 50hz
        print("Motor controller stopped")

class MotorDriver(MotorControllerHandler):
    def __init__(self, port=None, baudrate=115200, timeout=1):
        self.send_queue = Queue(15) #send items in this queue to motor controller 

        self.COM_HWID = "USB VID:PID=0483:5740"

        if not port:
            port = self.get_comm()
            if not port: # fuck
                raise OSError("serial comm port not found!!!!!!")

        super().__init__(port, baudrate, timeout, self.send_queue)

        self.speed = 0
        self.direction = 0
        self.turn_speed = 0

        self.motor_angles = [300, 180, 60]

    def __enter__(self):
        super().start(self.send_queue)
        return self

    def start(self):
        print("starting motor driver")
        self.__enter__()

    def get_target_motor_speeds(self):
        speeds = [
            int(
                linear_map(sin((i-self.direction) * pi/180) * self.speed + self.turn_speed, 0, 100, 0, self.MAX_SPEED),
            ) for i in self.motor_angles
        ]

        for i in range(len(speeds)):
            if speeds[i] > self.MAX_SPEED:
                speeds[i] = self.MAX_SPEED
            if speeds[i] < -self.MAX_SPEED:
                speeds[i] = -self.MAX_SPEED

        return speeds

    def get_comm(self):
        ports = comports()
        for p in ports:
            print(p.hwid)
            if p.hwid.startswith(self.COM_HWID):
                return p.device

    #add new motor movement into queue that sends data to controller. 
    #Return true if successfuly added into queue
    def send(self, speed=None, direction=None, turn_speed=None, thrower=None, servo_angle=None, servo_hold=None, callback=None):
        if speed is not None:
            self.speed = speed
            if self.speed > 100:
                self.speed = 100
            elif self.speed < 0:
                self.speed = 0
        if direction is not None:
            self.direction = direction
        if turn_speed is not None:
            self.turn_speed = turn_speed

        motor1, motor2, motor3 = self.get_target_motor_speeds()

        if servo_hold is not None:
            servo_hold = int(linear_map(servo_hold, 0, 100, 4450, 5))

        if servo_angle is not None:
            if servo_angle < 0:
                servo_angle = 0
            if servo_angle > 100:
                servo_angle = 100
            servo_angle = int(linear_map(servo_angle, 0, 100, 2200, 3500))

        if not self.send_queue.full(): #ignore when send queue is full
            self.send_queue.put_nowait((motor1, motor2, motor3, thrower, servo_angle, servo_hold, callback))
            return True

        return False

    def stop(self, callback=None):
        self.send_queue.put((0, 0, 0, 0, 0, 0, callback)) #Block until free slot to be sure that this data is sent to motors

    def close(self):
        print("stopping motor driver")
        self.stop()
        self.send_queue.put(STOP_CODE)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

if __name__ == "__main__":
    #callback functions must be globally accessable for example __main__.callback
    #because lambda doesn't have name, they can't be used
    def callback(x):
        print(x)


    with MotorDriver() as driver:
        driver.send(speed=10, direction=0, turn_speed=0, thrower=0, callback=callback) #add new data to send into controller
