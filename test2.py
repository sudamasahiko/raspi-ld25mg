import RPi.GPIO as GPIO
import time
import Adafruit_PCA9685
import sys

GPIO.setmode(GPIO.BOARD)
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)
RESOLUTION = 4096
PULSE_WIDTH_MS = 20.0
PULSE_FROM = RESOLUTION * 0.5 / PULSE_WIDTH_MS
PULSE_TO = RESOLUTION * 2.5 / PULSE_WIDTH_MS

ratio_angle = float(sys.argv[1]) / 180
pulse_target = int(PULSE_FROM + ratio_angle * (PULSE_TO - PULSE_FROM))

def get_pulse(angle):
    ratio_angle = angle / 180.0
    pulse_target = int(PULSE_FROM + ratio_angle * (PULSE_TO - PULSE_FROM))
    return pulse_target

pwm.set_pwm(15, 0, get_pulse(0))
time.sleep(1)

try:
    while True:
        pwm.set_pwm(15, 0, int(PULSE_FROM))
        time.sleep(1)
        pwm.set_pwm(15, 0, pulse_target)
        time.sleep(1)

except KeyboardInterrupt:
    pwm.set_pwm(15, 0, get_pulse(0))
    time.sleep(1)
    pass
