#!/usr/bin/env python3
#coding=utf-8
import time
import serial
from Rosmaster_Lib import Rosmaster
#from ipywidgets import interact
#import ipywidgets as widgets
bot = Rosmaster()
ser = serial.Serial(port='/dev/ttyS1',baudrate=9600,timeout=1)
# 控制PWM舵机 Control PWM steering gear  
def pwm_servo(S1):
    bot.set_pwm_servo(1, S1)
    return S1
bot.set_pwm_servo(1,97)
bot.set_pwm_servo(2,180)
while True:
    data = ser.readline()
    if data:
      data_decode = data.decode('utf-8')
      angle = 3*int(float(data_decode))+97
      print (angle)
      pwm_servo(angle)
