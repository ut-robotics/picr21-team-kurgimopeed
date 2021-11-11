from src.motor_driver import MotorDriver

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

    def run_logic(self, data):
        #if not self.enable:
        #    return

        # no balls -> spin
        if not len(data["balls"]):
            self.motor_driver.send(turn_speed=10)
            return

        # sort balls by distance
        ball_loc, ball_depth = sorted(data["balls"], key=lambda ball: ball[1])[0]

        # vague target ball location 
        # height is not considered as a constraint atm
        target = [0.028, 0.22]
        tolerance = [0.01, 0.01]
        #delta_target = [ball_coord - target_coord for ball_coord, target_coord in zip(closest, target)]

        # x is perpendicular to robot forward vector (horizontal)
        # y is parralel to robot forward vector (horizontal)
        # z is parralel to robot up vector (vertical)

        # TODO: scale turning speed with delta and depth
        l = ["turn right", "turn left", "forwards", "backwards"]
        for coord in range(2):
            delta = ball_loc[coord] - target[coord]
            if delta > tolerance[coord]:
                print(l[2 * coord])
                return
            elif delta < -tolerance[coord]:
                print(l[2 * coord + 1])
                return

        print("ball found")

        # pseudo
        # if goal not found
        #   rotate and revolve around ball, return
        #   TODO: implement rotation around arbitrary target whilst
        #   keeping forward vector pointed towards target
        # 
        # if goal not in forward vector tolerance
        #   rotate left/right with appropriate speed
