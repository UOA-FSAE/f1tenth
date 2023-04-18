import sys
import rclpy
from .SimulationServices import SimulationServices

def main():
    rclpy.init()

    services = SimulationServices('empty')
    
    print('Hi from simulation.')
    print(sys.argv)


if __name__ == '__main__':
    main()
