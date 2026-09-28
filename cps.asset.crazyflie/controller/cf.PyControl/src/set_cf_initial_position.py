import logging
import time
import argparse

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

def convert_to_full_uri(address):
    """Converts a short address into a full URI."""
    if address.startswith('radio://'):
        # Already a full URI: return it unchanged
        return address
    else:
        # Short addresses such as E1 or E2 are expanded into a full URI
        # Strip a leading 'E' and keep only the digits
        if address.startswith('E'):
            address = address[1:]
        return f'radio://0/80/2M/E7E7E7E7E{address}'

def parse_arguments():
    """Parses the command-line arguments."""
    parser = argparse.ArgumentParser(description='Set the initial position of a Crazyflie')
    parser.add_argument('--uri', type=str, default='E1',
                       help='Crazyflie address (default: E1; accepts short forms such as E1 or E2, or a full URI)')
    parser.add_argument('--x', type=float, default=0.0,
                       help='Initial X coordinate (default: 0.0)')
    parser.add_argument('--y', type=float, default=0.8,
                       help='Initial Y coordinate (default: 0.8)')
    parser.add_argument('--z', type=float, default=0.0,
                       help='Initial Z coordinate (default: 0.0)')
    return parser.parse_args()

def set_initial_position(scf, initial_x, initial_y, initial_z):
    cf = scf.cf
    # Only set the parameters once the connection has been established
    cf.param.set_value('kalman.initialX', str(initial_x))
    cf.param.set_value('kalman.initialY', str(initial_y))
    cf.param.set_value('kalman.initialZ', str(initial_z))
    # Reset the estimator so the position estimate starts from the new values
    cf.param.set_value('kalman.resetEstimation', '1')

    # Give the firmware a moment to apply them
    time.sleep(0.1)
    print(f"Set initial position to X={initial_x}, Y={initial_y}, Z={initial_z} and reset estimation.")

if __name__ == '__main__':
    # Parse the command-line arguments
    args = parse_arguments()
    
    # Convert the address to a URI
    full_uri = convert_to_full_uri(args.uri)
    
    logging.basicConfig(level=logging.INFO)

    # Initialise the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(full_uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected to Crazyflie!")
        set_initial_position(scf, args.x, args.y, args.z)
        print("Ready for takeoff.")
