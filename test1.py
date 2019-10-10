import RPi.GPIO as GPIO
import time
import Adafruit_PCA9685

GPIO.setmode(GPIO.BOARD)

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

try:
    while True:
        pwm.set_pwm(15, 0, 600)
        time.sleep(1)
        pwm.set_pwm(15, 0, 375)
        time.sleep(1)

except KeyboardInterrupt:
    pass
