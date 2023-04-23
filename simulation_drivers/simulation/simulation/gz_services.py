import sys
import time
import rclpy
from .simulation_services import SimulationServices
from ament_index_python.packages import get_package_share_directory
def main():
    rclpy.init()
    pkg_simulation = get_package_share_directory('simulation')

    services = SimulationServices('empty')
    
    print('Hi from simulation.')
    
    
    print(sys.argv)
    while True:
        services.spawn(sdf_filename=f'{pkg_simulation}/sdf/goal.sdf')    
        time.sleep(1)
        services.set_pose('cool_car', [1, 1, 1])
        time.sleep(1)
        services.delete_entity('cool_car', 2)
        time.sleep(1)
        services.spawn(sdf_filename=f'{pkg_simulation}/sdf/goal.sdf')
        time.sleep(1)
        services.reset()
        time.sleep(1)
    rclpy.spin(services)


if __name__ == '__main__':
    main()
