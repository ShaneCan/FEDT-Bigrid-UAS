import time
from crazyflie_webots_examples.crazyflie_client import Crazyflie


def main():

    cf = Crazyflie(cf_id=0)

    cf.takeoff(height=1.0, secs=3)
    print("Taking off...")

    time.sleep(3)

    cf.goto(position=[0.0, 2.0, 2.0], secs=3)
    print("Going to position [0.0, 2.0, 2.0]...")

    time.sleep(3)

    cf.land(secs=3)
    print("Landing...")

    time.sleep(3)


if __name__ == "__main__":
    main()
