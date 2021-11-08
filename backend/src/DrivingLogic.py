from src.motor_driver import MotorDriver

class DrivingLogic():
    def __init__(self, motor_driver: MotorDriver):
        self.motor_driver = motor_driver
        self.enable = False # controlled from web interface

    def run_logic(self, data):
        if not self.enable:
            return

        if len(data["balls"]):
            pass
            #print(data["balls"][0])
        # ebig logic :.--DD
        # spin until ball center
        # drive 2 ball
        # 
        #self.motor_driver.send(turn_speed=20)
