from franky import Robot, JointMotion

# 1. Connect and Clear Errors
robot = Robot("192.168.1.12")
robot.recover_from_errors() # Essential if it hit a limit while going sideways

# 2. Safety First: Set speed to very slow (10%)
robot.relative_dynamics_factor = 0.1 

# 3. Standard "Ready" Joint Configuration
# These values are taken directly from your library examples
ready_joints = [0.0, 0.0, 0.0, -2.2, 0.0, 2.2, 0.7]

# Standard Franka Emika "Home" Position
#factory_home = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
factory_home = [0.36845, -0.07430, 0.12793, -0.49720, 0.50683, 0.50370, 0.49214 ]

print("Moving robot to default center position...")
try:
    # Use JointMotion to untangle the arm
    robot.move(JointMotion(ready_joints))
    #robot.move(JointMotion(factory_home))
    print("Robot reset successfully.")
except Exception as e:
    print(f" Reset failed: {e}")


