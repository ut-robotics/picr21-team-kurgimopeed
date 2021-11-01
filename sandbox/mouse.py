import evdev

MOUSE_NAME = "Logitech G502 HERO Gaming Mouse"

dev = next((evdev.InputDevice(d) for d in evdev.list_devices() if evdev.InputDevice(d).name == MOUSE_NAME))

x = 0
y = 0
for e in dev.read_loop():
    if e.type == evdev.ecodes.EV_REL:
        if e.code == evdev.ecodes.REL_X:
            x += e.value
        if e.code == evdev.ecodes.REL_Y:
            y += e.value
        
        print(f"{x} {y}")

