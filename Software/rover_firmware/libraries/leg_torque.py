# leg_torque.py
# Servo torque control library

import time
import serial

# CHANGE THIS to your Mega's serial port
SERIAL_PORT = "COM3"
BAUD_RATE = 115200

_serial_instance = None

def _get_serial():
    global _serial_instance
    if _serial_instance is None:
        _serial_instance = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
    return _serial_instance

def send_angle(board, channel, angle):
    ser = _get_serial()
    angle = max(0, min(180, int(angle)))
    cmd = f"{board},{channel},{angle}\n"
    ser.write(cmd.encode("ascii"))
    time.sleep(0.01)

def send_disable(board, channel):
    ser = _get_serial()
    cmd = f"{board},{channel},DISABLE\n"
    ser.write(cmd.encode("ascii"))
    time.sleep(0.01)

# ===== Wiring Mapping =====
LEG_CHANNELS = {
    1: {"coxa": ("A",0),  "femur": ("A",1),  "tibia": ("A",2)},
    2: {"coxa": ("A",6),  "femur": ("A",13), "tibia": ("A",9)},
    3: {"coxa": ("A",3),  "femur": ("A",7),  "tibia": ("A",10)},
    4: {"coxa": ("B",11), "femur": ("B",14), "tibia": ("B",15)},
    5: {"coxa": ("B",10), "femur": ("B",13), "tibia": ("B",12)},
    6: {"coxa": ("B",14), "femur": ("B",9),  "tibia": ("B",15)},
}

# ===== Standing angles =====
STANDING_ANGLES = {
    (1,"coxa"):90, (1,"femur"):165, (1,"tibia"):165,
    (2,"coxa"):90, (2,"femur"):15,  (2,"tibia"):15,
    (3,"coxa"):90, (3,"femur"):165, (3,"tibia"):165,
    (4,"coxa"):90, (4,"femur"):15,  (4,"tibia"):15,
    (5,"coxa"):90, (5,"femur"):15,  (5,"tibia"):15,
    (6,"coxa"):90, (6,"femur"):165, (6,"tibia"):165,
}

CURRENT_ANGLES = {k:v for k,v in STANDING_ANGLES.items()}

def _set_torque(leg, joint, enable):
    board, ch = LEG_CHANNELS[leg][joint]
    if enable:
        angle = CURRENT_ANGLES[(leg,joint)]
        send_angle(board, ch, angle)
    else:
        send_disable(board, ch)

def init_standing_pose():
    for leg in range(1,7):
        for joint in ("coxa","femur","tibia"):
            _set_torque(leg, joint, True)

# ===== Public Torque APIs =====

# Leg 1
def LegOneCoxaTorque(e): _set_torque(1,"coxa",e)
def LegOneFemurTorque(e): _set_torque(1,"femur",e)
def LegOneTibiaTorque(e): _set_torque(1,"tibia",e)

# Leg 2
def LegTwoCoxaTorque(e): _set_torque(2,"coxa",e)
def LegTwoFemurTorque(e): _set_torque(2,"femur",e)
def LegTwoTibiaTorque(e): _set_torque(2,"tibia",e)

# Leg 3
def LegThreeCoxaTorque(e): _set_torque(3,"coxa",e)
def LegThreeFemurTorque(e): _set_torque(3,"femur",e)
def LegThreeTibiaTorque(e): _set_torque(3,"tibia",e)

# Leg 4
def LegFourCoxaTorque(e): _set_torque(4,"coxa",e)
def LegFourFemurTorque(e): _set_torque(4,"femur",e)
def LegFourTibiaTorque(e): _set_torque(4,"tibia",e)

# Leg 5
def LegFiveCoxaTorque(e): _set_torque(5,"coxa",e)
def LegFiveFemurTorque(e): _set_torque(5,"femur",e)
def LegFiveTibiaTorque(e): _set_torque(5,"tibia",e)

# Leg 6
def LegSixCoxaTorque(e): _set_torque(6,"coxa",e)
def LegSixFemurTorque(e): _set_torque(6,"femur",e)
def LegSixTibiaTorque(e): _set_torque(6,"tibia",e)
