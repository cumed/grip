# -*- coding: utf-8 -*-
"""
Created on Thur June 21 12:58:00

@author: ATI-2 Pavan Gurudath
"""
import RPi.GPIO as GPIO
from time import sleep

LedPin = 7    # pin7
GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
GPIO.setup(LedPin, GPIO.OUT)   # Set LedPin's mode is output
GPIO.output(LedPin, GPIO.HIGH) # Set LedPin high(+3.3V) to turn on led

def startHeat(heating_time):
    print('Heater on')
    GPIO.output(LedPin, GPIO.HIGH)  # led on
    sleep(heating_time)
    print('Heater off')
    GPIO.output(LedPin, GPIO.LOW) # led off
    sleep(0.1)

def destroy():
  GPIO.output(LedPin, GPIO.LOW)   # led off
  GPIO.cleanup()                  # Release resource

if __name__ == "__main__":
    LedPin = 7    # pin7
    GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
    GPIO.setup(LedPin, GPIO.OUT)   # Set LedPin's mode is output
    GPIO.output(LedPin, GPIO.HIGH) # Set LedPin high(+3.3V) to turn on led
    while True:
        GPIO.output(LedPin, GPIO.HIGH)
        sleep(1)
        GPIO.output(LedPin, GPIO.LOW)
        sleep(1)
        
