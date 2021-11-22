from cv2 import line
from src.motor_driver import MotorDriver
from src.Tools import linear_map
import math
from simple_pid import PID

# logic notes
# no balls -> spin.
# balls -> pick one, which one?
#   closest.
# somehow mark the ball? blob ids?
#   no obvious online solutions.
#   assign random ids to all balls, cache balls (loc and id), find
#   new match for all new balls.
#   idiotic for current purposes, just find closest ball always.
#   we can mark balls later for webview. combined with status ("going to ball #5")
#   it may prove useful for debugging
# spin until ball is vertically center to camera
#   this can be immensely optimized by driving towards the ball at the same time (1)
#   but it works for a simple solution
# go to ball until dist is l/e constant
# circle ball until goal is vertical center of cam (does camera offset matter?)
#   ideally we want center to thrower but maybe neglible
# estimate basket distance using magic
# throw
#   ignore the thrown ball somehow? dont want to follow thrown ball

# algo idea (1)
# with proper localization we can cache ball locations (and assume they stay there)
# whilst we spin and drive towards them. this allows us to scan the entire field
# for closer balls/opponent whilst also working towards scoring points

class DrivingLogic():
    def __init__(self, motor_driver: MotorDriver):
        self.motor_driver = motor_driver
        self.enable = False # controlled from web interface

        self.state = "" # useful for frontend status

        # in metres, unused, a cone/triangle would better than
        # a rectangle - depth should be taken into account
        # because its hard to turn towards distant balls precisely
        self.BALL_CENTER_TOLERANCE = 0.01 # in m

        self.pid = PID(0.6, 0, 0, setpoint=0)
        self.pid.output_limits = (-20, 20)

    def run_logic(self, data):
        if not self.enable:
            return

        self.motor_driver.stop()

        # no balls -> spin
        if not len(data["balls"]):
            self.motor_driver.send(turn_speed=3)
            return

        # sort balls by distance and get closest
        ball_loc, ball_depth = sorted(data["balls"], key=lambda ball: ball[1])[0]

        # x is perpendicular to robot forward vector (horizontal)
        # y is parralel to robot forward vector (horizontal)
        # z is parralel to robot up vector (vertical)

        #print(self.pid.tunings)

        depth_target = 0.25
        alpha = math.atan(ball_loc[0] / ball_loc[1]) * 180.0 / math.pi
        new = self.pid(alpha)
        target_speed = min((ball_depth - depth_target) * 100, 30)
        self.motor_driver.send(direction=0, turn_speed=new, speed=target_speed)
        print(alpha)
        print(new)
        print(ball_depth - depth_target)
        print("")

        return


        print("")
        return
        if alpha > turning_tolerance or alpha < -turning_tolerance:
            turn_speed = linear_map(alpha, -50, 40, 15, -15)
            print(turn_speed)
            #print(alpha)
            self.motor_driver.send(turn_speed=turn_speed)
            return

        # driving
        depth_tolerance = 0.02 #
        depth_target = 0
        print("found:", ball_depth)

        # pseudo
        # if goal not found
        #   rotate and revolve around ball, return
        #   TODO: implement rotation around arbitrary target whilst
        #   keeping forward vector pointed towards target
        # 
        # if goal not in forward vector tolerance
        #   rotate left/right with appropriate speed
