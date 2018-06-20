# -*- coding: utf-8 -*-
"""
Created on Tue Jun 19 14:53:32 2018

@author: ATI-2 Pavan Gurudath
"""
import RPi.GPIO as GPIO
from time import sleep

LedPin = 7    # pin11
GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
GPIO.setup(LedPin, GPIO.OUT)   # Set LedPin's mode is output
GPIO.output(LedPin, GPIO.HIGH) # Set LedPin high(+3.3V) to turn on led

def startHeat(heating_time):
    print('Heater on')
    GPIO.output(LedPin, GPIO.HIGH)  # led on
    sleep(heating_time)
    print('Heater off')
    GPIO.output(LedPin, GPIO.LOW) # led off

def destroy():
  GPIO.output(LedPin, GPIO.LOW)   # led off
  GPIO.cleanup()                  # Release resource

