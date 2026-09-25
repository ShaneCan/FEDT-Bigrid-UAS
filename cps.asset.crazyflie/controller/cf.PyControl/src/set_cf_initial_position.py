import logging
import time
import argparse

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

def convert_to_full_uri(address):
    """将简短地址转换为完整的URI格式"""
    if address.startswith('radio://'):
        # 如果已经是完整URI，直接返回
        return address
    else:
        # 假设简短地址是E1、E2等格式，转换为完整URI
        # 移除可能的'E'前缀，只取数字部分
        if address.startswith('E'):
            address = address[1:]
        return f'radio://0/80/2M/E7E7E7E7E{address}'

def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='设置Crazyflie的初始位置')
    parser.add_argument('--uri', type=str, default='E1',
                       help='Crazyflie的地址 (默认: E1, 支持E1、E2等简短格式或完整URI)')
    parser.add_argument('--x', type=float, default=0.0,
                       help='初始X坐标 (默认: 0.0)')
    parser.add_argument('--y', type=float, default=0.8,
                       help='初始Y坐标 (默认: 0.8)')
    parser.add_argument('--z', type=float, default=0.0,
                       help='初始Z坐标 (默认: 0.0)')
    return parser.parse_args()

def set_initial_position(scf, initial_x, initial_y, initial_z):
    cf = scf.cf
    # 确保成功连接后设置参数
    cf.param.set_value('kalman.initialX', str(initial_x))
    cf.param.set_value('kalman.initialY', str(initial_y))
    cf.param.set_value('kalman.initialZ', str(initial_z))
    # 重置估计器，使位置估计以新值为初始
    cf.param.set_value('kalman.resetEstimation', '1')

    # 等待一点时间，让固件生效
    time.sleep(0.1)
    print(f"Set initial position to X={initial_x}, Y={initial_y}, Z={initial_z} and reset estimation.")

if __name__ == '__main__':
    # 解析命令行参数
    args = parse_arguments()
    
    # 转换URI格式
    full_uri = convert_to_full_uri(args.uri)
    
    logging.basicConfig(level=logging.INFO)

    # 初始化底层通信
    cflib.crtp.init_drivers()

    with SyncCrazyflie(full_uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected to Crazyflie!")
        set_initial_position(scf, args.x, args.y, args.z)
        print("Ready for takeoff.")
