import telnetlib
import time
import subprocess

tn = telnetlib.Telnet("127.0.0.1",port=4444,timeout=10)

# tn.write("reset\n".encode('utf-8'))
# time.sleep(1)
tn.write("reset halt\n".encode('utf-8'))
time.sleep(1)
tn.write("flash write_image erase /home/pi/Downloads/MailBox_16MHz/MailBox.elf\n".encode('utf-8'))
time.sleep(2.9)

# p = subprocess.Popen(['./main data.txt'], shell = True)
tn.write("resume\n".encode('utf-8'))
print('resume')
time.sleep(1)
# tn.write("halt\n".encode('utf-8'))
# time.sleep(1)
# tn.write("flash write_image erase /home/pi/Downloads/MailBox_16MHz/MailBox.elf\n".encode('utf-8'))
# time.sleep(1)
time.sleep(15)
tn.write("exit\n".encode('utf-8'))
time.sleep(1)
# p.wait()
