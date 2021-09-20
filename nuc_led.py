import os

#echo "02 02 64 01 02" > /proc/acpi/nuc_led

class NucLED():
    TYPE_POWER = 1
    TYPE_RING = 2

    POWER_OFF = 0
    POWER_BLUE = 1
    POWER_AMBER = 2

    RING_OFF = 0
    RING_CYAN = 1
    RING_PINK = 2
    RING_YELLOW = 3
    RING_BLUE = 4
    RING_RED = 5
    RING_GREEN = 6
    RING_WHITE = 7

    MODE_OFF = 0
    MODE_ON = 4
    MODE_BLINK_1HZ = 1
    MODE_BLINK_05HZ = 5
    MODE_BLINK_025HZ = 2
    MODE_FADE_1HZ = 3
    MODE_FADE_05HZ = 7
    MODE_FADE_025HZ = 6

    def __init__(self):
        self._PATH = "/proc/acpi/nuc_led"
        
        self.parse(self.TYPE_POWER, self.read(self.TYPE_POWER))
        self.parse(self.TYPE_RING, self.read(self.TYPE_RING))
    
    def get_types(self):
        return [self.TYPE_POWER, self.TYPE_RING]

    def get_ring_colors(self):
        return [self.RING_OFF, self.RING_CYAN, self.RING_PINK, self.RING_YELLOW,
                self.RING_BLUE, self.RING_RED, self.RING_GREEN, self.RING_WHITE]

    def get_power_colors(self):
        return [self.POWER_OFF, self.POWER_BLUE, self.POWER_AMBER]

    def get_modes(self):
        return [self.MODE_OFF, self.MODE_ON, 
                self.MODE_BLINK_1HZ, self.MODE_BLINK_05HZ, self.MODE_BLINK_025HZ, 
                self.MODE_FADE_1HZ, self.MODE_FADE_05HZ, self.MODE_FADE_025HZ]

    def set_led(self, TYPE, brightness=None, mode=None, color=None):
        if TYPE == self.TYPE_POWER:
            if brightness is not None:
                self.POWER_BRIGHTNESS = brightness
            if mode is not None:
                self.POWER_MODE = mode
            if color is not None:
                self.POWER_COLOR = color
        elif TYPE == self.TYPE_RING:
            if brightness is not None:
                self.RING_BRIGHTNESS = brightness
            if mode is not None:
                self.RING_MODE = mode
            if color is not None:
                self.RING_COLOR = color
        self.write(TYPE)
                
    def write(self, TYPE):
        if TYPE == self.TYPE_POWER:
            self.echo("02 %02d %02d %02d %02d"%(TYPE, self.POWER_BRIGHTNESS, self.POWER_MODE, self.POWER_COLOR))
            
        elif TYPE == self.TYPE_RING:
            self.echo("02 %02d %02d %02d %02d"%(TYPE, self.RING_BRIGHTNESS, self.RING_MODE, self.RING_COLOR))

    def read(self, TYPE):
        self.echo("01 %02d 00 00 00"%TYPE)

        return os.popen("cat "+self._PATH).read().split("\n")[0]

    def echo(self, data):
        os.system("echo \"%s\" > %s"%(data, self._PATH))
        
    def parse(self, TYPE, data):
        #get/set, type, brightness, mode, color
        _, BRIGHTNESS, MODE, COLOR = [int(i) for i in data.split(" ")]
        
        if TYPE == self.TYPE_POWER:
            if BRIGHTNESS > 64 or BRIGHTNESS < 0:
                raise Exception("Parsed out of boundaries nuc led brightness: %d"%(BRIGHTNESS))
            self.POWER_BRIGHTNESS = BRIGHTNESS

            if MODE in self.get_modes():
                self.POWER_MODE = MODE
            else:
                raise Exception("Parsed unknown nuc led mode: %d"%(MODE))

            if COLOR in self.get_power_colors():
                self.POWER_COLOR = COLOR
            else:
                raise Exception("Parsed unknown nuc led power led color: %d"%(COLOR))

        elif TYPE == self.TYPE_RING:
            if BRIGHTNESS > 64 or BRIGHTNESS < 0:
                raise Exception("Parsed out of boundaries nuc led brightness: %d"%(BRIGHTNESS))
            self.RING_BRIGHTNESS = BRIGHTNESS

            if MODE in self.get_modes():
                self.RING_MODE = MODE
            else:
                raise Exception("Parsed unknown nuc led mode: %d"%(MODE))

            if COLOR in self.get_ring_colors():
                self.RING_COLOR = COLOR
            else:
                raise Exception("Parsed unknown nuc led ring led color: %d"%(COLOR))

if __name__ == "__main__":
    import time
    led = NucLED()

    while True:
        for i in led.get_ring_colors():
            led.set_led(led.TYPE_RING, color=i, mode=led.MODE_ON)
            time.sleep(0.1)