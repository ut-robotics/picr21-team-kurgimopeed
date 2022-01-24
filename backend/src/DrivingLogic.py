from cv2 import line
from src.MotorDriver import MotorDriver
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
        self.timer = None

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

        self.thrower_timeout = 1

        self.spin_speed = 30
        self.spin_timeout = 5
        self.spin_current_time = 0
        self.min_balls = 1

    def callback(self, data):
        *motor_speeds, thrower_speed, ball_event = data
        if ball_event:
            print("PALL")

    def get_goal_data(self, data):
        return data["blue_goal"] if self.target_goal == GoalDetector.ID_BLUE else data["pink_goal"]

    # called externally from referee server or front end
    def start(self, target_goal=None):
        self.enable = True
        if target_goal is not None:
            self.target_goal = target_goal
        self.state = self.spin
        self.motor_driver.send(thrower=0, servo_hold=0, speed=0)

    def stop(self):
        self.enable = False
        #time.sleep(0.1)
        #self.motor_driver.stop()
    
    def spin(self, data):
        l = len(data["balls"])

        if self.min_balls <= l:
            self.state = self.drive
        else:
            self.motor_driver.send(speed=0, turn_speed=self.spin_speed)

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
        self.motor_driver.send(direction=alpha, turn_speed=new_turn_speed, speed=target_speed)
        #print(f"{alpha=}")
        #print(f"{new_turn_speed=}")
        #print(f"{(ball_depth - self.depth_target)=}")

        if ball_depth - self.depth_target < 0.02:
            self.state = self.get_ball

    def get_ball(self, data):
        def exit():
            self.motor_driver.send(servo_hold=0)
            self.timer = None

        def callback(data):
            *motor_speeds, thrower_speed, ball_event = data
            print("Got Ball", ball_event)
            '''if ball_event:
                self.state = self.spin_goal
                print("Got Ball")
                exit()'''
        
        if self.timer is None:
            self.timer = time.time()
        if time.time()-self.timer > 2:
            self.state = self.spin
            exit()
            return 
        self.motor_driver.send(direction=0, turn_speed=0, speed=30, servo_hold=100, callback=callback)
            
    def spin_goal(self, data):
        goal = self.get_goal_data(data)
        if goal:
            self.state = self.throw
        self.motor_driver.send(direction=0, turn_speed=20, speed=0)

    def throw(self, data):
        goal = self.get_goal_data(data)
        if not goal:
            self.state = self.spin_goal
            return

        goal_loc, goal_depth = goal
        goal_alpha = (math.atan(goal_loc[0] / goal_loc[1]) * 180.0 / math.pi)
        #print(f"{goal_alpha=}")
        new_turn_speed = self.drive_pid(goal_alpha)

        if goal_alpha < 2:
            if self.timer is None:
                self.timer = time.time()
            self.motor_driver.send(servo_hold=100)


        if self.timer is not None:
            if time.time() - self.timer > self.thrower_timeout:
                self.timer = None
                self.state = self.spin
                self.motor_driver.send(servo_hold=0, thrower=0)
                return

        thrower_speed = self.thrower.get_speed(goal_depth)
        self.motor_driver.send(direction=0, turn_speed=new_turn_speed, speed=0, thrower=45000)

    def run_logic(self, data):
        if not self.enable or not self.state:
            return

        self.state(data)
        print(self.state)