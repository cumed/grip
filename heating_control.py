# -*- coding: utf-8 -*-
"""
Created on Mon Jul 23 10:40:30 2018
new bend
@author: ATI-2 Pavan Gurudath
"""

import RPi.GPIO as GPIO
from time import sleep

LedPin = 7    # #4
heatTime_off = 0.1
GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
GPIO.setup(LedPin, GPIO.OUT)   # Set LedPin's mode is output
GPIO.output(LedPin, GPIO.LOW) # Set LedPin high(+3.3V) to turn on led

def startHeat(heating_time):
    # Edit this function to send signals for the heating system to start and stop. 
    # Currently the function contains Led ON and OFF. 
    
#    print('Heater on')
    GPIO.output(LedPin, GPIO.HIGH)  # led on
    sleep(heating_time)
#    print('Heater off')
    GPIO.output(LedPin, GPIO.LOW) # led off
    sleep(heatTime_off)

def destroy():
  GPIO.output(LedPin, GPIO.LOW)   # led off
  GPIO.cleanup()                  # Release resource

if __name__ == "__main__":
    LedPin = 7    # pin7 
    GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
    GPIO.setup(LedPin, GPIO.OUT)   # Set LedPin's mode is output
    GPIO.output(LedPin, GPIO.LOW) # Set LedPin high(+3.3V) to turn on led
    while True:
        GPIO.output(LedPin, GPIO.HIGH)
        sleep(0.5)
        GPIO.output(LedPin, GPIO.LOW)
        sleep(0.5)
        GPIO.output(LedPin, GPIO.HIGH)
        sleep(0.5)
        GPIO.output(LedPin, GPIO.LOW)
        sleep(0.5)