from cv2 import line
from src.motor_driver import MotorDriver
from src.Tools import linear_map
import math, time
from simple_pid import PID
from src.GoalDetector import GoalDetector
from src.ThrowerTraining import ThrowerTraining

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

        self.thrower = ThrowerTraining()

        # possible states
        # spin: look for balls
        # drive: drive towards ball whilst turning
        # circle: circle around ball looking for target goal
        # throw: throw and correct
        self.state = None

        # drive turn_speed pid
        self.drive_pid = PID(0.6, 0, 0, setpoint=0)
        self.drive_pid.output_limits = (-20, 20)

        # circle turn_speed pid
        self.circle_turn_pid = PID(0.6, 0.4, 0.2, setpoint=0)
        self.circle_turn_pid.output_limits = (-20, 20)
        self.depth_target = 0.25

        # circle direction_speed pid (for goal)
        self.circle_dir_pid = PID(0.6, 0.4, 0.2, setpoint=0)
        self.circle_dir_pid.output_limits = (-20, 20)

        self.target_goal = GoalDetector.ID_PINK

        self.thrower_timeout = 3.0
        self.current_thrower_time = 0.0

        self.spin_speed = 30
        self.spin_timeout = 5
        self.spin_current_time = 0
        self.min_balls = 1

    # called externally from referee server or front end
    def init(self):
        self.state = self.spin
    
    def spin(self, data):
        l = len(data["balls"])
        if self.min_balls <= l:
            self.state = self.drive
        else:
            self.motor_driver.send(speed=0, turn_speed=self.spin_speed, thrower=0)

    def drive(self, data):
        if not len(data["balls"]):
            self.state = self.spin
            return

        ball_loc, ball_depth = sorted(data["balls"], key=lambda ball: ball[1])[0]

        # x is perpendicular to robot forward vector (horizontal)
        # y is parralel to robot forward vector (horizontal)
        # z is parralel to robot up vector (vertical)

        #print(self.pid.tunings)

        alpha = (math.atan(ball_loc[0] / ball_loc[1]) * 180.0 / math.pi)
        new_turn_speed = self.drive_pid(alpha)
        target_speed = min((ball_depth - self.depth_target) * 100, 50)
        self.motor_driver.send(direction=alpha, turn_speed=new_turn_speed, speed=target_speed, thrower=0)
        #print(f"{alpha=}")
        #print(f"{new_turn_speed=}")
        #print(f"{(ball_depth - self.depth_target)=}")

        if ball_depth - self.depth_target < 0.02:
            self.state = self.circle

    def circle(self, data):
        if not len(data["balls"]):
            self.state = self.spin
            return
            
        ball_loc, ball_depth = sorted(data["balls"], key=lambda ball: ball[1])[0]
        alpha = (math.atan(ball_loc[0] / ball_loc[1]) * 180.0 / math.pi)
        new_turn_speed = self.circle_turn_pid(alpha)
        delta = max(min(300 * (ball_depth - self.depth_target), 45), -45)
        #print(f"{delta=}")

        goal = data["blue_goal"] if self.target_goal == GoalDetector.ID_BLUE else data["pink_goal"]
        if not goal:
            self.circle_dir_pid.reset()
            self.motor_driver.send(direction=90 - delta, turn_speed=new_turn_speed, speed=40)
            return

        goal_loc, goal_depth = goal
        goal_alpha = (math.atan(goal_loc[0] / goal_loc[1]) * 180.0 / math.pi)
        print(f"{alpha=}")
        print(f"{goal_alpha=}")
        new_dir_speed = self.circle_dir_pid(goal_alpha)

        self.motor_driver.send(direction=90-delta, turn_speed=new_turn_speed, speed=new_dir_speed)
        if abs(alpha) < 3 and abs(goal_alpha) < 3:
            self.state = self.throw

    def throw(self, data):
        goal = data["blue_goal"] if self.target_goal == GoalDetector.ID_BLUE else data["pink_goal"]
        if not goal:
            self.state = self.spin
            return

        goal_loc, goal_depth = goal
        goal_alpha = (math.atan(goal_loc[0] / goal_loc[1]) * 180.0 / math.pi)
        #print(f"{goal_alpha=}")
        new_turn_speed = self.drive_pid(goal_alpha)

        if self.current_thrower_time == 0.0:
            self.current_thrower_time = time.time()

        if time.time() - self.current_thrower_time > self.thrower_timeout:
            self.current_thrower_time = 0.0
            self.state = self.spin
            return

        self.motor_driver.send(direction=goal_alpha, turn_speed=new_turn_speed, speed=12, thrower=self.thrower.get_speed(goal_depth))

    def run_logic(self, data):
        if not self.enable or not self.state:
            return

        self.state(data)

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
