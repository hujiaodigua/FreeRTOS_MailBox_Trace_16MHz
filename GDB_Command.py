#!/usr/bin/env python3
# coding=utf-8

import gdb
import time
import os
import subprocess

gdb.execute('target remote localhost:3333')
gdb.execute('monitor reset')
gdb.execute('monitor halt')
gdb.execute('load')
# print('恢复target的运行')
# p = subprocess.Popen(['./USB_Sample data.txt'],shell = True)
# os.system('./USB_Sample data.txt')

# time.sleep(0.1)
gdb.execute('monitor resume') # 这个跑到USB传输程序前面了
# p.poll()
gdb.execute('set confirm off') # 关闭请求询问，让推出跳过yes/No
gdb.execute('quit')
# os._exit(0)
