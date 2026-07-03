# Simply tests if moveitpy is recognised

import rclpy
from moveit.planning import MoveItPy

def main():
    rclpy.init()
    print("Initialising moveitpy")
    robot = MoveItPy(node_name="rldp5_moveit_py")
    print("Instance created sucessfully")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
