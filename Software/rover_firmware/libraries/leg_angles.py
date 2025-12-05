# leg_angles.py
# Absolute and relative angle control

from .leg_torque import (
    LEG_CHANNELS, STANDING_ANGLES, CURRENT_ANGLES, send_angle
)

def _abs_angle(leg, joint, angle):
    angle = max(0, min(180, float(angle)))
    board, ch = LEG_CHANNELS[leg][joint]
    send_angle(board, ch, int(angle))
    CURRENT_ANGLES[(leg,joint)] = angle

def _rel_angle(leg, joint, delta):
    base = CURRENT_ANGLES[(leg,joint)]
    _abs_angle(leg, joint, base + delta)

# ===== APIs =====

# Leg 1
def LegOneCoxaAbsoluteAngle(a): _abs_angle(1,"coxa",a)
def LegOneCoxaRelativeAngle(d): _rel_angle(1,"coxa",d)
def LegOneFemurAbsoluteAngle(a): _abs_angle(1,"femur",a)
def LegOneFemurRelativeAngle(d): _rel_angle(1,"femur",d)
def LegOneTibiaAbsoluteAngle(a): _abs_angle(1,"tibia",a)
def LegOneTibiaRelativeAngle(d): _rel_angle(1,"tibia",d)

# Leg 2
def LegTwoCoxaAbsoluteAngle(a): _abs_angle(2,"coxa",a)
def LegTwoCoxaRelativeAngle(d): _rel_angle(2,"coxa",d)
def LegTwoFemurAbsoluteAngle(a): _abs_angle(2,"femur",a)
def LegTwoFemurRelativeAngle(d): _rel_angle(2,"femur",d)
def LegTwoTibiaAbsoluteAngle(a): _abs_angle(2,"tibia",a)
def LegTwoTibiaRelativeAngle(d): _rel_angle(2,"tibia",d)

# Leg 3
def LegThreeCoxaAbsoluteAngle(a): _abs_angle(3,"coxa",a)
def LegThreeCoxaRelativeAngle(d): _rel_angle(3,"coxa",d)
def LegThreeFemurAbsoluteAngle(a): _abs_angle(3,"femur",a)
def LegThreeFemurRelativeAngle(d): _rel_angle(3,"femur",d)
def LegThreeTibiaAbsoluteAngle(a): _abs_angle(3,"tibia",a)
def LegThreeTibiaRelativeAngle(d): _rel_angle(3,"tibia",d)

# Leg 4
def LegFourCoxaAbsoluteAngle(a): _abs_angle(4,"coxa",a)
def LegFourCoxaRelativeAngle(d): _rel_angle(4,"coxa",d)
def LegFourFemurAbsoluteAngle(a): _abs_angle(4,"femur",a)
def LegFourFemurRelativeAngle(d): _rel_angle(4,"femur",d)
def LegFourTibiaAbsoluteAngle(a): _abs_angle(4,"tibia",a)
def LegFourTibiaRelativeAngle(d): _rel_angle(4,"tibia",d)

# Leg 5
def LegFiveCoxaAbsoluteAngle(a): _abs_angle(5,"coxa",a)
def LegFiveCoxaRelativeAngle(d): _rel_angle(5,"coxa",d)
def LegFiveFemurAbsoluteAngle(a): _abs_angle(5,"femur",a)
def LegFiveFemurRelativeAngle(d): _rel_angle(5,"femur",d)
def LegFiveTibiaAbsoluteAngle(a): _abs_angle(5,"tibia",a)
def LegFiveTibiaRelativeAngle(d): _rel_angle(5,"tibia",d)

# Leg 6
def LegSixCoxaAbsoluteAngle(a): _abs_angle(6,"coxa",a)
def LegSixCoxaRelativeAngle(d): _rel_angle(6,"coxa",d)
def LegSixFemurAbsoluteAngle(a): _abs_angle(6,"femur",a)
def LegSixFemurRelativeAngle(d): _rel_angle(6,"femur",d)
def LegSixTibiaAbsoluteAngle(a): _abs_angle(6,"tibia",a)
def LegSixTibiaRelativeAngle(d): _rel_angle(6,"tibia",d)
