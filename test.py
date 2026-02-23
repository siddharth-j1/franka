from franky import *
import sys

# 1. Connect using the native RT Kernel
try:
    robot = Robot("192.168.1.15")
    print(" Successfully connected to the robot!")
except Exception as e:
    print(f"Connection failed: {e}")
    sys.exit(1)

# 2. Set strict speed limits (10% of max speed)
robot.relative_dynamics_factor = 0.10

# 3. Create the 10cm movement command
# This moves 0.10 meters (10cm) along the +X axis (Forward out of the gripper)
motion = CartesianMotion(Affine([0.10, 0.0, 0.0]), ReferenceType.Relative)

# 4. Execute
print(" Moving 10cm forward...")
robot.move(motion)
print(" Move complete!")