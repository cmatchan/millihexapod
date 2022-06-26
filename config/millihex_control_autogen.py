#!/usr/bin/env python3
import sys

def main():
    """Autogenerates a joint controller yaml file for millihex robot.
    Enables easy changes to be made to PID values."""
    
    # Change PID values for joint controller here
    p = 0.1
    i = 0.0
    d = 0.0

    # Millihex leg and joint count
    num_legs = 6
    joints_per_leg = 3

    # Write a yaml joint controller file
    with open("millihex_control.yaml", "w") as f:
        # Write joint_state_controller publisher
        f.write("millihex:\n"\
                "    # Publish all joint states -----------------------------------\n"\
                "    joint_state_controller:\n"\
                "        type: joint_state_controller/JointStateController\n"\
                "        publish_rate: 50\n\n"\
                "    # Position Controllers ---------------------------------------\n")

        # Write joint_position_controller publishers
        for k in range(num_legs):
            k = k + 1
            f.write(f"    # Leg {k}\n")

            for j in range(joints_per_leg):
                j = j + 1
                f.write(f"    leg{k}_joint{j}_position_controller:\n"\
                        f"        type: effort_controllers/JointPositionController\n"\
                        f"        joint: leg{k}_joint{j}\n"\
                        f"        pid: {{p: {p}, i: {i}, d: {d}}}\n")

            f.write("\n")


if __name__ == '__main__':
    sys.exit(main())