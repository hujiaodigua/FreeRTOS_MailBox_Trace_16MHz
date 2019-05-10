#!/usr/bin/env python3
# coding=utf-8
'''
运行脚本前记得运行一下pluseview或sigrok-cli保证固件上传
使用arm-none-eabi-gdb -q -x script.py xx.elf 
'''
# import gdb
import time
import datetime
import os
import shutil
import subprocess
import sys
import multiprocessing
import usb.core
import usb.util
from vcd import VCDWriter


def Captured_GPIO(e):
    
    # global file_object 
    # global Capturedfile_Name 
    # global Capturedfile_Format 
    # global CapturedGPIO_Name 
    # global CapturedGPIO_Format 
    global CapturedTimes 
    
    # gdb.execute('target remote localhost:3333')
    # gdb.execute('monitor reset')
    # gdb.execute('monitor halt')
    # gdb.execute('load')

    for i in range(0,int(CapturedTimes)):
        e.wait()
       #  e.set()

        time.sleep(0.25)    # 把USB传输数据前的那段时间等过去,不能让GPIO的中断捕获跑到USB传输前面去
        # gdb.execute('monitor resume')


        # e.clear()
        # print('wait e.is_set()--' + str(e.is_set()))
        # time.sleep(0.01)
        # CapturedGPIO_Command = ("./GPIO_Interrupt " + str(i) + CapturedGPIO_Name + CapturedGPIO_Format) 
        # CapturedGPIO_Command = ("./GPIO_Interrupt " + "mailbox_timestamp.txt")

        # response_GPIO = subprocess.getstatusoutput(CapturedGPIO_Command)  
        os.system('./GPIO_Interrupt mailbox_timestamp.txt ')     # 把这一句放到USB采集脚本里面:执行,毕竟GPIO捕获是可以等待信号到来进行捕获


def USB_Sample_OS(e):
    # e.wait()
    subprocess.Popen(['./USB_Sample data.txt'], shell = True)

def GDB_Command_OS(e):
    # gdb.execute('target remote localhost:3333')
    # gdb.execute('monitor reset')
    # gdb.execute('monitor halt')
    # gdb.execute('load')
    # print('恢复target的运行')
    # gdb.execute('monitor resume') # 这个跑到USB传输程序前面了
    time.sleep(0.5)
    subprocess.Popen(['arm-none-eabi-gdb -q -x GDB_Command.py MailBox.elf'], shell = True)

if __name__ == "__main__":
    
    # file_object = open(str(sys.argv[2]) + '.txt', 'w')
    # Capturedfile_Name = '-' + str(sys.argv[1])
    # Capturedfile_Format = '.vcd' # 这个在这个脚本里目前没使用20190313
    # CapturedGPIO_Name = '-GPIO_Timestamp'
    # CapturedGPIO_Format = '.txt'
    CapturedTimes = 1#sys.argv[3] #GPIO_Interrupt目前在有边沿到来后一次捕获1100个时间戳
    
    e = multiprocessing.Event()
    task_USB_Sample_OS = multiprocessing.Process(name = 'block', target = USB_Sample_OS, args = (e,))    # 在这个进程里用subprocess.Popen方法非阻塞外部调用GPIO_Interrupt程序捕捉定期触发的GPIO
    task_GDB_Command_OS = multiprocessing.Process(name = 'block', target = GDB_Command_OS, args = (e,))

    task_USB_Sample_OS.start()
    task_GDB_Command_OS.start()

    # gdb.execute('monitor resume')   
    
    # task_Captured_GPIO = multiprocessing.Process(name = 'block', target = Captured_GPIO, args = (e,))
    # task_USB_Sample = multiprocessing.Process(name = 'block', target = USB_Sample, args = (e,))
    # task_GDB_Command = multiprocessing.Process(name = 'block', target = GDB_Command, args = (e,))
    
    # gdb.execute('target remote localhost:3333')
    # gdb.execute('monitor reset')
    # gdb.execute('monitor halt')
    # gdb.execute('load')
    # gdb.execute('monitor resume')

    # time.sleep(5)

    # task_USB_Sample.start()
    
    # task_Captured_GPIO.start()
    
    # time.sleep(5)
    
    # e.set()
    # time.sleep(0.3)    # 等到pyusb_fx2_samples.py跑到采集前的位置
    
    
    # task_GDB_Command.start()
    # gdb.execute('monitor resume')

    # subprocess.getstatusoutput('arm-none-eabi-gdb -q -x gdb_script.py MailBox.elf')
