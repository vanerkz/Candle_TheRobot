/*
 * SCSerial.h
 * 飞特串行舵机硬件接口层程序
 * 日期: 2019.7.22
 * 作者: 
 */

#include <iostream>
#include "../include/feetechlib/SCSerial.h"
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
bool writecond;
SCSerial::SCSerial()
{
	IOTimeOut = 100;
	fd = -1;
}

SCSerial::SCSerial(u8 End):SCS(End)
{
	IOTimeOut = 100;
	fd = -1;
}

SCSerial::SCSerial(u8 End, u8 Level):SCS(End, Level)
{
	IOTimeOut = 100;
	fd = -1;
}

bool SCSerial::begin(int baudRate, const char* serialPort)
{
writecond=true;
	if(fd != -1){
		close(fd);
		fd = -1;
	}
	//printf("servo port:%s\n", serialPort);
    if(serialPort == NULL)
		return false;
    fd = open(serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd == -1){
		perror("open:");
        return false;
	}
    fcntl(fd, F_SETFL, FNDELAY);
    tcgetattr(fd, &orgopt);
    tcgetattr(fd, &curopt);
    speed_t CR_BAUDRATE;
    switch(baudRate){
    case 9600:
        CR_BAUDRATE = B9600;
        break;
    case 19200:
        CR_BAUDRATE = B19200;
        break;
    case 38400:
        CR_BAUDRATE = B38400;
        break;
    case 57600:
        CR_BAUDRATE = B57600;
        break;
    case 115200:
        CR_BAUDRATE = B115200;
        break;
    case 500000:
        CR_BAUDRATE = B500000;
        break;
    case 1000000:
        CR_BAUDRATE = B1000000;
        break;
    default:
		CR_BAUDRATE = B115200;
        break;
    }
    cfsetispeed(&curopt, CR_BAUDRATE);
    cfsetospeed(&curopt, CR_BAUDRATE);

	printf("serial speed %d\n", baudRate);
    //Mostly 8N1
    curopt.c_cflag &= ~PARENB;
    curopt.c_cflag &= ~CSTOPB;
    curopt.c_cflag &= ~CSIZE;
    curopt.c_cflag |= CS8;
    curopt.c_cflag |= CREAD;
    curopt.c_cflag |= CLOCAL;//disable modem statuc check
    cfmakeraw(&curopt);//make raw mode
    curopt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    if(tcsetattr(fd, TCSANOW, &curopt) == 0){
        return true;
    }else{
		perror("tcsetattr:");
		return false;
	}
}

int SCSerial::setBaudRate(int baudRate)
{ 
    if(fd==-1){
		return -1;
	}
    tcgetattr(fd, &orgopt);
    tcgetattr(fd, &curopt);
    speed_t CR_BAUDRATE;
    switch(baudRate){
    case 9600:
        CR_BAUDRATE = B9600;
        break;
    case 19200:
        CR_BAUDRATE = B19200;
        break;
    case 38400:
        CR_BAUDRATE = B38400;
        break;
    case 57600:
        CR_BAUDRATE = B57600;
        break;
    case 115200:
        CR_BAUDRATE = B115200;
        break;
    case 230400:
        CR_BAUDRATE = B230400;
	break;
    case 500000:
        CR_BAUDRATE = B500000;
        break;
    default:
        break;
    }
    cfsetispeed(&curopt, CR_BAUDRATE);
    cfsetospeed(&curopt, CR_BAUDRATE);
    return 1;
}

int SCSerial::readSCS(unsigned char *nDat, int nLen)
{
if(writecond)
{	
    int fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);

    time.tv_sec = 0;
    time.tv_usec = IOTimeOut*1000;

    //使用select实现串口的多路通信
    fs_sel = select(fd+1, &fs_read, NULL, NULL, &time);
    if(fs_sel){
		fs_sel = read(fd, nDat, nLen);
        //printf("nLen = %d fs_sel = %d\n", nLen, fs_sel);
        return fs_sel;
    }else{
        //printf("serial read fd read return 0\n");
        return 0;
}
}
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
writecond=false;
int data = write(fd, nDat, nLen);
writecond=true;
	return data;//write(serial_port, data, data_size
}

int SCSerial::writeSCS(unsigned char bDat)
{
writecond=false;
int data = write(fd, &bDat, 1);
writecond=true;
	return data;
}

void SCSerial::rFlushSCS()
{

	tcflush(fd, TCIFLUSH);
}

void SCSerial::wFlushSCS()
{
}

void SCSerial::end()
{
	fd = -1;
	close(fd);
}
