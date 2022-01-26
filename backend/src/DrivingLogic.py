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
        self.last_state = None

        self.timer = None
        self.timer2 = None

        # drive turn_speed pid
        self.spin_pid = PID(0.3, 0.1, 0.075, setpoint=0)
        self.spin_pid.output_limits = (-20, 20)

        self.spin_ball_pid = PID(0.5, 0.5, 0.1, setpoint=0)
        self.spin_ball_pid.output_limits = (-20, 20)

        self.depth_target = 0.5

        self.target_goal = GoalDetector.ID_BLUE

        self.thrower_timeout = 2

        self.spin_speed = 10
        self.spin_timeout = 5
        self.spin_current_time = 0
        self.min_balls = 1

        self.approach_direction = 0
        self.ball_lost_right = True 
        self.goal_lost_right = True 
        self.last_error = None

    def callback(self, data):
        *motor_speeds, thrower_speed, ball_event = data
        if ball_event:
            self.state = self.spin_goal

    def get_goal_data(self, data):
        return data["blue_goal"] if self.target_goal == GoalDetector.ID_BLUE else data["pink_goal"]

    # called externally from referee server or front end
    def start(self, target_goal=None):
        print("start:", target_goal)
        self.enable = True
        if target_goal is not None:
            self.target_goal = target_goal
        self.state = self.spin
        self.motor_driver.stop()

    def referee_stop(self):
        print("referee stop")
        self.stop()
        self.motor_driver.stop()

    def stop(self):
        self.enable = False
        #time.sleep(0.1)
        #self.motor_driver.stop()
    
    def spin(self, data):
        l = len(data["balls"])

        if self.min_balls <= l:
            self.state = self.drive
        else:
            self.motor_driver.send(thrower=0,speed=0, turn_speed=-10 if self.ball_lost_right else 10, servo_hold=0)
            #self.motor_driver.send(thrower=0,speed=0, turn_speed=10, servo_hold=0, callback=self.callback)

    def drive(self, data):
        if not len(data["balls"]):
            self.state = self.spin
            return

        ball_loc, ball_depth = data["balls"][0]

        # x is perpendicular to robot forward vector (horizontal)
        # y is parralel to robot forward vector (horizontal)
        # z is parralel to robot up vector (vertical)

        #print(self.pid.tunings)

        alpha = (math.atan(ball_loc[0] / ball_loc[1]) * 180.0 / math.pi)
        self.ball_lost_right = False if alpha < 0 else True
        new_turn_speed = self.spin_pid(alpha)
        error = ball_depth - self.depth_target
        target_speed = max(min((error) * 100, 50), 25)
        print(target_speed)

        self.motor_driver.send(direction=5*alpha, turn_speed=new_turn_speed, speed=target_speed, thrower=0)#, callback=self.callback)
        #print(f"{alpha=}")
        print(f"{new_turn_speed=}")
        #print(f"{(ball_depth - self.depth_target)=}")

        if error < 0 and alpha < 1/ball_depth:
            self.state = self.get_ball

    def spin_ball(self, data):
        if not len(data["balls"]):
            self.state = self.spin
            return

        ball_loc, ball_depth = data["balls"][0]

        alpha = (math.atan(ball_loc[0] / ball_loc[1]) * 180.0 / math.pi)
        print(alpha, 1/ball_depth)
        self.ball_lost_right = False if alpha < 0 else True

        if abs(alpha) < 1/ball_depth:
            if self.timer is None:
                self.timer = time.time()
            self.motor_driver.send(speed=0, turn_speed=0, servo_hold=0)

            if time.time()-self.timer > 0.5:
                self.state = self.get_ball
                return
        else:
            self.timer = None

        new_turn_speed = self.spin_ball_pid(alpha)
        self.motor_driver.send(turn_speed=new_turn_speed, speed=0, direction=0, servo_hold=0)#, callback=self.callback)


    def get_ball(self, data):

        alpha = 0
        if len(data["balls"]):
            ball_loc, ball_depth = data["balls"][0]

            alpha = (math.atan(ball_loc[0] / ball_loc[1]) * 180.0 / math.pi)
            self.ball_lost_right = False if alpha < 0 else True
        new_turn_speed = self.spin_pid(alpha)

        def callback(data):
            *motor_speeds, thrower_speed, ball_event = data
            if ball_event:
                print("Got Ball")
                self.state = self.spin_goal

        if self.timer is None:
            self.timer = time.time()
        if time.time()-self.timer > 3:
            if self.state == self.get_ball:
                self.state = self.spin_goal#self.spin
            return 
        self.motor_driver.send(direction=5*alpha, turn_speed=0, speed=20, servo_hold=100)#, callback=callback)

    def spin_goal(self, data):
        goal = self.get_goal_data(data)
        if goal:
            self.state = self.aim

        self.motor_driver.send(speed=0, turn_speed=-15 if self.goal_lost_right else 15, servo_hold=0)
        #self.motor_driver.send(turn_speed=10, speed=0, callback=callback)

    def aim(self, data):
        def callback(data):
            *motor_speeds, thrower_speed, ball_event = data
            if not ball_event:
                self.state = self.spin
        goal = self.get_goal_data(data)
        if not goal:
            self.state = self.spin_goal
            return

        goal_loc, goal_depth = goal
        goal_alpha = (math.atan(goal_loc[0] / goal_loc[1]) * 180.0 / math.pi)
        print(f"{goal_alpha=}")
        self.goal_lost_right = False if goal_alpha < 0 else True

        new_turn_speed = self.spin_ball_pid(goal_alpha)
        if abs(goal_alpha) < 2:
            if self.timer is None:
                self.timer = time.time()
            self.motor_driver.send(speed=0, turn_speed=0, servo_hold=0)

            if time.time()-self.timer > 0.25:
                self.state = self.okse
                return
        else:
            self.timer = None
            if goal_depth < 0.5:
                self.approach_direction = 180
            elif goal_depth > 1.5:
                self.approach_direction = 0
            approach_speed = min(20, max(goal_depth*20, 30))
            self.motor_driver.send(direction=self.approach_direction, speed=approach_speed, turn_speed=new_turn_speed, servo_hold=0, callback=callback)
        
    def okse(self, data):
        print("okse")
        if self.timer is None:
            self.timer = time.time()
        if time.time()-self.timer>0.2:
            self.state = self.throw
            self.motor_driver.send(servo_hold=0, turn_speed=0, speed=0)
            return
        self.motor_driver.send(servo_hold=-15, turn_speed=0, speed=0)

    def throw(self, data):
        current_time = time.time()
        if self.timer is None:
            self.timer = current_time

        goal = self.get_goal_data(data)
        if not goal:
            self.state = self.spin_goal
            return

        goal_loc, goal_depth = goal
        
        thrower_hold = 0
        if current_time-self.timer > 1:
            self.state = self.spin
            self.motor_driver.send(thrower=0, servo_hold=0)
            return
        elif current_time-self.timer > 0.5:
            thrower_hold = 100
        thrower_speed, thrower_angle = self.thrower.get_speed(goal_depth)
        self.motor_driver.send(turn_speed=0, speed=0, direction=0, thrower=thrower_speed, servo_angle=thrower_angle, servo_hold=thrower_hold)

    def run_logic(self, data):
        if not self.enable or not self.state:
            return

        print(self.state)

        self.last_state = str(self.state)
        self.state(data)

        if str(self.state) != str(self.last_state):
            self.spin_pid.reset()
            self.spin_ball_pid.reset()
            self.timer = None
            self.timer2 = None
            self.last_error = None