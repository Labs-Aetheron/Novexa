# main_rover.py
# Python-side control entry point

from libraries.leg_torque import (
    init_standing_pose,
    LegOneCoxaTorque,
    LegOneFemurTorque,
    LegOneTibiaTorque,
)
from libraries.leg_angles import (
    LegOneCoxaAbsoluteAngle,
    LegOneCoxaRelativeAngle,
)

def main():
    # Move robot to standing pose
    init_standing_pose()
    print("Standing pose set.")

    # Example absolute movement
    LegOneCoxaAbsoluteAngle(90)
    print("Leg1 Coxa moved to 90° (abs).")

    # Example relative movement
    LegOneCoxaRelativeAngle(+10)
    print("Leg1 Coxa adjusted by +10°")

    # Disable femur/tibia torque
    LegOneFemurTorque(False)
    LegOneTibiaTorque(False)
    print("Leg1 Femur/Tibia torque disabled.")

if __name__ == "__main__":
    main()
