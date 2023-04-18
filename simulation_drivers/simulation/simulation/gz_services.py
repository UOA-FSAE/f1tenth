import sys
import rclpy
from .simulation_services import SimulationServices

def main():
    rclpy.init()

    services = SimulationServices('empty')
    
    print('Hi from simulation.')
    print(sys.argv)

    rclpy.spin(services)


if __name__ == '__main__':
    main()
