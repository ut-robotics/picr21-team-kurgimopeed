import evdev
from evdev import InputDevice, ecodes

LG_MOUSE = "Logitech G502 HERO Gaming Mouse"
RZ_MOUSE = "Razer Razer DeathAdder Chroma"

lg_mouse = next((evdev.InputDevice(d) for d in evdev.list_devices() if evdev.InputDevice(d).name == LG_MOUSE))

rz_mouse = next((
    InputDevice(d)
    for d in evdev.list_devices()
    if InputDevice(d).name == RZ_MOUSE
    and ("EV_REL", 2) in list(InputDevice(d).capabilities(verbose=True).keys())
))

x = 0
y = 0
for e in lg_mouse.read_loop():
    if e.type == evdev.ecodes.EV_REL:
        if e.code == evdev.ecodes.REL_X:
            x += e.value
        if e.code == evdev.ecodes.REL_Y:
            y += e.value
        
        print(f"{x} {y}")
