# test3.py

import time, sys, math, threading
import RPi.GPIO as GPIO
import Adafruit_PCA9685

# constants
RESOLUTION = 4096
PULSE_WIDTH_MS = 20.0
PULSE_FROM = RESOLUTION * 0.5 / PULSE_WIDTH_MS
PULSE_TO = RESOLUTION * 2.5 / PULSE_WIDTH_MS
PORT_MOTOR1 = 15
PORT_MOTOR2 = 14
PORT_MOTOR3 = 13

# calibration for the motors
fn = 'calibration.cfg'
with open(fn, 'r') as f:
    lines = f.readlines()
    f.close()

if len(lines) != 3:
    sys.exit()

calib = []
for line in lines:
    calib.append(float(line))

GPIO.setmode(GPIO.BOARD)
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

def get_pulse(angle):
    ratio_angle = angle / 180.0
    pulse_target = int(PULSE_FROM + ratio_angle * (PULSE_TO - PULSE_FROM))
    return pulse_target

# homing
pwm.set_pwm(PORT_MOTOR1, 0, get_pulse(calib[0]))
pwm.set_pwm(PORT_MOTOR2, 0, get_pulse(calib[1]))
pwm.set_pwm(PORT_MOTOR3, 0, get_pulse(calib[2]))
ports = [PORT_MOTOR1, PORT_MOTOR2, PORT_MOTOR3]
time.sleep(1)

deg_m1_last = 0
deg_m2_last = 0
deg_m3_last = 0

def set_angle(idx_motor, angle):
    resolution = 100
    for i in range(resolution):
        ratio = math.sin(math.pi * 0.5 * float(i+1) / resolution)
        computed = calib[idx_motor] - deg_m1_last - ratio * (angle-deg_m1_last)
        pwm.set_pwm(ports[idx_motor], 0, get_pulse(computed))
        time.sleep(0.01)

angle_new = float(sys.argv[1])
t1 = threading.Thread(target=set_angle, args=(0, angle_new))
t2 = threading.Thread(target=set_angle, args=(1, angle_new))
t3 = threading.Thread(target=set_angle, args=(2, angle_new))
t1.start()
t2.start()
t3.start()
t1.join()
t2.join()
t3.join()
deg_m1_last = angle_new
deg_m2_last = angle_new
deg_m3_last = angle_new
time.sleep(1)

angle_new = 0
t1 = threading.Thread(target=set_angle, args=(0, angle_new))
t2 = threading.Thread(target=set_angle, args=(1, angle_new))
t3 = threading.Thread(target=set_angle, args=(2, angle_new))
t1.start()
t2.start()
t3.start()
t1.join()
t2.join()
t3.join()
deg_m1_last = angle_new
deg_m2_last = angle_new
deg_m3_last = angle_new
time.sleep(1)

# set_angle(0, 0.0)
# set_angle(1, 0.0)
# set_angle(2, 0.0)
# time.sleep(1)

# homing
# pwm.set_pwm(PORT_MOTOR1, 0, get_pulse(calib[0]))
# pwm.set_pwm(PORT_MOTOR2, 0, get_pulse(calib[1]))
# pwm.set_pwm(PORT_MOTOR3, 0, get_pulse(calib[2]))
# time.sleep(1)

