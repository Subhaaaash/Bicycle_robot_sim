from simulator import BicycleSimulator
from control_algorithms import keyboard_control # , simple_avoidance_control

def main():
    sim = BicycleSimulator()
    # Uncomment one of these to choose control algorithm:

    # Manual keyboard control:
    sim.run(keyboard_control)

    # Simple LiDAR-based avoidance control:
   # sim.run(simple_avoidance_control)

if __name__ == "__main__":
    main()
