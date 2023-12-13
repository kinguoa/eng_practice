# Untitled - By: Administrator - 周一 十一月 29 2021
import time,image,math
import sensor#引入感光元件的模块
from pyb import Pin
import json
from pyb import UART

from pyb import Servo

#舵机IO设置
Servo_x = Servo(2) # P7
Servo_y = Servo(1) # P8

Servo_x.angle(0)
Servo_y.angle(0)

# x y 轴舵机控制函数封装
def Set_Servo_x_angle(temp):
   if (temp > -50 and temp <46) :
      Servo_x.angle(temp)
   elif temp < -50:
      Servo_x.angle(-50)
   else:
      Servo_x.angle(50)

def Set_Servo_y_angle(temp):
   if (temp > -46 and temp <46) :
      Servo_y.angle(temp)
   elif temp < -46:
      Servo_y.angle(-46)
   else:
      Servo_y.angle(46)



threshold_index = 0 # 0 for red, 1 for green, 2 for blue
green_threshold   = (59,97,-46,-20,51,76)

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds = [(30, 40, -40, -20, 30, 50)] # generic_red_thresholds
# 设置摄像头
sensor.reset()#初始化感光元件
sensor.set_pixformat(sensor.RGB565)#设置为彩色 RGB565: 彩色
sensor.set_framesize(sensor.QVGA)#设置图像的大小 sensor.QVGA: 320x240，sensor.VGA: 640x480 HVGA:480×320
sensor.set_windowing(29,0,260,300)
#sensor.set_auto_gain(True) # 自动增益开启（True）
#sensor.set_auto_whitebal(True) #自动白平衡开启
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
# 开灯
from pyb import LED
led = LED(1) # 红led
led.on()#亮
led.off()

block_xv = 0
block_yv = 0
block_lx = 0
block_ly = 0

sensor.set_pixformat(sensor.RGB565)#设置为彩色 RGB565: 彩色



time.sleep_ms(100)
#pin9.value(0)
#time.sleep_ms(500)
sensor.skip_frames()#跳过n张照片，在更改设置后，跳过一些帧，等待感光元件变稳定。nr

clock = time.clock()
flag_dw = True


class PID_Inc:
    """Standard PID Controller"""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pre_error = 0
        self.sum_error = 0

    def set_kp(self, kp):
        self.kp = kp

    def set_ki(self, ki):
        self.ki = ki

    def set_kd(self, kd):
        self.kd = kd

    def update(self, target, current):
        error = target - current
        self.sum_error += error

        # Calculate PID terms
        p_term = self.kp * error
        i_term = self.ki * self.sum_error
        d_term = self.kd * (error - self.pre_error)

        # Combine terms
        output = p_term + i_term + d_term

        # Update previous error for the next iteration
        self.pre_error = error

        return output


    #pid 设置
Pid_x = PID_Inc(0.09, 0.00, 1.8)
Pid_y = PID_Inc(0.09, 0.00, 1.8)
Pid_xv = PID_Inc(0.53, 0.000, 0)
Pid_yv = PID_Inc(0.53, 0.000, 0)
cnt =0
while(flag_dw):
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)#lens_corr(1.8)畸变矫正
    #img.draw_cross(140,114,size=5, color=(255,255,255))
    #img.draw_circle(140,114,10, color=(255,0,0))

    blobs = img.find_blobs([green_threshold])

    if blobs: #如果找到了目标颜色
            cnt =0
            for blob in blobs:
            #迭代找到的目标颜色区域
                # Draw a rect around the blob.
              #  img.draw_rectangle(blob[0:4]) # rect
                #用矩形标记出目标颜色区域
                img.draw_cross(blob.cx(), blob.cy(), color=(255,0,0)) # cx, cy

                #在目标颜色区域的中心画十字形标记
                block_xv = (blob.cx() - block_lx) / 10
                block_yv = (blob.cy() - block_ly) / 10
                block_lx = blob.cx()
                block_ly = blob.cy()

                if abs(300 - blob.cx()) > 20  or abs(300 - blob.cy()) > 20:
                    x_ev = Pid_x.update(140, blob.cx())  #pid计算
                    y_ev = Pid_y.update(114, blob.cy())  #pid计算

                    x_ange = Pid_xv.update(x_ev, block_xv)
                    y_ange = Pid_yv.update(y_ev, block_yv)
                    print("x_ev, blob.cx():\r\n", x_ev, blob.cx())
                    print("y_ev, blob.cy():\r\n", y_ev, blob.cy())
                    Set_Servo_x_angle(6+x_ange)
                    Set_Servo_y_angle(y_ange+4)
                else:
                    Set_Servo_x_angle(6)
                    Set_Servo_y_angle(4)
                time.sleep_ms(10)


   #没发现目标回正
    else:
       cnt = cnt+1
       if cnt > 50:
           Set_Servo_x_angle(6)
           Set_Servo_y_angle(4)
           cnt = 0

