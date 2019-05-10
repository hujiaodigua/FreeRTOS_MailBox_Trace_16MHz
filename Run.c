/*************************************************************************
	> File Name: Run.c
	> Author: 
	> Mail: 
	> Created Time: 2019年05月09日 星期四 17时43分07秒
 ************************************************************************/

#include<stdio.h>
#include<stdlib.h>
#include<pthread.h>
#include<time.h>
#include<unistd.h>

void *GDB_Command(void *arg)
{
    // system("arm-none-eabi-gdb -q -x GDB_Command.py MailBox.elf");
    system("python3 Telnet_Command.py");
}

void *USB_Sample(void *arg)
{
    usleep(4000000);
    // usleep(500000);
    system("./main data.txt");
}

int main(void)
{
    
    // popen("arm-none-eabi-gdb -q -x GDB_Command.py MailBox.elf","r");
    // system("./main data.txt");
    pthread_t GDB_Command_tid;
    pthread_t USB_Sample_tid;

    pthread_create(&USB_Sample_tid, NULL, USB_Sample, NULL);
    pthread_create(&GDB_Command_tid, NULL, GDB_Command, NULL);
    // pthread_create(&USB_Sample_tid, NULL, USB_Sample, NULL);

    pthread_join(GDB_Command_tid, NULL);
    pthread_join(USB_Sample_tid, NULL);

}
