from franky import Robot

# Connect to your Franka
robot = Robot("192.168.1.12")

# Read the physical joint angles
angles = robot.current_state.q

print("\n🎯 SUCCESS! Copy these 7 numbers exactly:")
print(f"PERFECT_JOINTS = {list(angles)}")