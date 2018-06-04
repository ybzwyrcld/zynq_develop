#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define GPIO_DIR_OUT 1
#define GPIO_DIR_IN 0


uint8_t TxBuffer0[5]  = {0x00, 0x00, 0x40, 0x00, 0x00}; //
uint8_t TxBuffer1[5]  = {0x01, 0x00, 0x41, 0x00, 0x20}; //
uint8_t TxBuffer2[5]  = {0x02, 0x22, 0x18, 0x41, 0x30}; //
uint8_t TxBuffer3[5]  = {0x03, 0x00, 0x00, 0x00, 0xff}; //  fsc
uint8_t TxBuffer4[9]  = {0x0e, 0x78, 0xff, 0x00, 0x00, 0x29, 0xd0, 0x36, 0x9d}; //  freq_reg

/*
 *  *pin_num:(n-1)*32+m
 *  eg:
 *		gpio8_3 is (8-1)*32+m
 */
static int gpio_init(int pin_num)
{
	printf("gpio init the pin(%d)\n",pin_num);
	char data[128];
	sprintf(data,"echo %d > /sys/class/gpio/export",pin_num);
	system(data);
	return 0;
}

static int gpio_set_direction(int pin_num,char *dir)
{
	char data[128];
	printf("gpio set the pin(%d) direction is %s\n",pin_num,dir);
	sprintf(data,"echo %s > /sys/class/gpio/gpio%d/direction",dir,pin_num);
	system(data);
	return 0;
}

static int gpio_set_value(int pin_num,int value)
{
	char data[128];
	sprintf(data,"echo %d > /sys/class/gpio/gpio%d/value",value,pin_num);
	system(data);
	return 0;
}


void write_delay(int loop)
{
	for(loop;loop>0;loop--)
	{
		usleep(1);
	}
}

void write_enable(void)
{
	printf("write ad9957 reg enable\n");
	gpio_set_value(960, 1);
	write_delay(1);
	gpio_set_value(960, 0);
}

static int ad9957_config(void)
{
	int ret = 0;
	int fd = open("/dev/emio_spi",O_RDWR);
	if( fd == -1){
		printf("Can't open emio_spi device!\n");
		return ret;
    }
	else{
        printf("/dev/emio_spi open success!\n");
	}
//	printf("%s\n",__func__);
	ret = write(fd, TxBuffer0, sizeof(TxBuffer0));
	write_enable();
	ret = write(fd, TxBuffer1, sizeof(TxBuffer1));
	write_enable();
	ret = write(fd, TxBuffer2, sizeof(TxBuffer2));
//	write_enable();
	ret = write(fd, TxBuffer3, sizeof(TxBuffer3));
	write_enable();
	ret = write(fd, TxBuffer4, sizeof(TxBuffer4));
	write_enable();

	close(fd);

	return ret;
}

int main(int argc,char *argv[])
{
	int ret = 0;

	/* pull low i/o updata pin */
	gpio_init(960);
	gpio_set_direction(960, "out");
	gpio_set_value(960, 0);

	if(argc == 2)
		TxBuffer1[3] = 0x08; /* pdclk endable */

	ret = ad9957_config();

    return ret;

}

