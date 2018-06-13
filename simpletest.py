# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685


# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 190 # Min pulse length out of 4096
servo_max = 2100 # Max pulse length out of 4096
servo_mid = 500
# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    print(pulse)
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
counter =1
def defineangle(angle,from_low,from_high,to_low,to_high):
    pulse_angle = (angle-from_low)*(to_high-to_low)/(from_high-from_low)+to_low
    return int(pulse_angle)



flag_c=1
while flag_c==1:
    flag= input('Enter 0 or 1 for zero position or random position')
    if flag==0:
        for channel in range(0,8):
            pwm.set_pwm(channel,0,servo_min)
##            time.sleep(1)
    else:
        for channel in range(0,8):
            pwm.set_pwm(channel,0,400+channel*10)
##            time.sleep(1)

    time.sleep(2)

