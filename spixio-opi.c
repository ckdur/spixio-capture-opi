/* Include GPIO and SPIDEV linux kernel headers */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "spixio.h"

// CKDUR: Change me for CS pin (manual handled)
#define GPIO_CS 19
// CKDUR: Change me for any dev device
static const char *device = "/dev/spidev0.0";
static uint8_t buffer[SPI_DEVICE_BUFFER_SIZE] = {0};
static uint8_t rbuffer[SPI_DEVICE_BUFFER_SIZE] = {0};

uint32_t GSPI_INPUTS;
uint32_t GSPI_ADDR;
uint32_t GSPI_OUTPUTS;
uint32_t GSPI_WORD;
uint8_t  GSPI_DONE;
uint32_t GSPI_MAXOBJ;
uint32_t GSPI_SPI_ADDR_BITS;
char GSPI_NAME[500];

static int ftHandle;
static int fd_gpio;

// **********************************************
// ** GPIO AUXILIAR LIBRARY
// ** Extracted from rpi examples:

#define IN  0
#define OUT 1
 
#define LOW  0
#define HIGH 1

static int GPIOExport(int pin)
{
#define BUFFER_MAX 3
	char buffer[BUFFER_MAX];
	ssize_t bytes_written;
	int fd;
 
	fd = open("/sys/class/gpio/export", O_WRONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open export for writing!\n");
		return(-1);
	}
 
	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
	write(fd, buffer, bytes_written);
	close(fd);
	return(0);
}
 
static int GPIOUnexport(int pin)
{
	char buffer[BUFFER_MAX];
	ssize_t bytes_written;
	int fd;
 
	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open unexport for writing!\n");
		return(-1);
	}
 
	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
	write(fd, buffer, bytes_written);
	close(fd);
	return(0);
}
 
static int GPIODirection(int pin, int dir)
{
	static const char s_directions_str[]  = "in\0out";
 
#define DIRECTION_MAX 35
	char path[DIRECTION_MAX];
	int fd;
 
	snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
	fd = open(path, O_WRONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open gpio direction for writing!\n");
		return(-1);
	}
 
	if (-1 == write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3)) {
		fprintf(stderr, "Failed to set direction!\n");
		return(-1);
	}
 
	close(fd);
	return(0);
}
 
static int GPIOOpenValue(int pin, int mode)
{
#define VALUE_MAX 30
	char path[VALUE_MAX];
	char value_str[3];
	int fd;
 
	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, mode);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open gpio value!\n");
		return(-1);
	} 
 
	return fd;
}

static int GPIOCloseValue(int fd)
{
	close(fd);
	return 0;
}
 
static int GPIORead(int fd)
{
	char value_str[3];
	if (-1 == read(fd, value_str, 3)) {
		fprintf(stderr, "Failed to read value!\n");
		return(-1);
	}
 
	return(atoi(value_str));
}
 
static int GPIOWrite(int fd, int value)
{
	static const char s_values_str[] = "01";
 
	if (1 != write(fd, &s_values_str[LOW == value ? 0 : 1], 1)) {
		fprintf(stderr, "Failed to write value!\n");
		return(-1);
	}
	return(0);
}
// **********************************************

// **********************************************
// ** SPI KERNEL AUXILIARS

#define pabort(exp) {fprintf(stderr,"%s:%d:%s(): status(%s) \
\n",__FILE__, __LINE__, __FUNCTION__, exp); return 0;}

static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = SPI_CLOCK_RATE_HZ;
static uint16_t delay = 0;

// **********************************************

int clogb2(int value)
{
	int 	i;
	int clogb = 0;
	for(i = 0; (1<<i) < value; i = i + 1)
		clogb = i + 1;
	return clogb;
}

int read_write_single_word(uint32_t address, uint32_t sizeaddr, uint32_t *data, uint32_t databits)
{
	int ret;
	int i, n, s;
	
	/* CS_High + Write command + Address */
	buffer[0] = GSPI_TASK_RW << 6;	/* Write command (2bit, 6-bit displaced)*/
	n = 0; s = 5;
	buffer[0] = 0xBA;
	buffer[1] = 0xAB;
	for(i = ((int)sizeaddr-1); i >= 0; i--)
	{
        buffer[n] |= ((address >> i) & 0x1) << s;
        if(s == 0) {s = 7; n++; buffer[n] = 0;} else s--;
	}

	for(i = ((int)databits-1); i >= 0; i--)
	{
        buffer[n] |= (((*data) >> i) & 0x1) << s;
        if(s == 0) {s = 7; n++; buffer[n] = 0;} else s--;
	}
	
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)buffer,
		.rx_buf = (unsigned long)rbuffer,
		.len = (GSPI_SPI_COMM_READ_BITS+GSPI_SPI_READ_SEND_BITS)/8 + 1,	// byte adjust
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	
	// Set Intended CS to 0
	if (-1 == GPIOWrite(fd_gpio, LOW))
		return(0);

	// Write and Read the data
	ret = ioctl(ftHandle, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
	
	// Set Intended CS to 1
	if (-1 == GPIOWrite(fd_gpio, HIGH))
		return(0);

	n = 0; s = 7 - GSPI_SPI_COMM_READ_BITS + 1;
	while(s < 0) {s += 8; n++;}
	(*data) = 0;
	for(i = ((int)databits-1); i >= 0; i--)  /*IS NOT FUCKING LITTLE ENDIAN*/
	{
        (*data) |= ((rbuffer[n] >> s) & 0x1) << i;
        if(s == 0) {s = 7; n++;} else s--;
	}

	/* Dummy Bits, for issue aditional clk cycles */
	tr.len = 1;
	ret = ioctl(ftHandle, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	return 1;
}

int write_single_word(uint32_t address, uint32_t sizeaddr, uint32_t data, uint32_t databits)
{
	int ret;
	int i, n, s;

	/* CS_High + Write command + Address */
	buffer[0] = 0xBA;
	buffer[1] = 0xAB;
	buffer[0] = GSPI_TASK_WRITE << 6;	/* Write command (2bit, 6-bit displaced)*/
	n = 0; s = 5;
	for(i = ((int)sizeaddr-1); i >= 0; i--)
	{
        buffer[n] |= ((address >> i) & 0x1) << s;
        if(s == 0) {s = 7; n++; buffer[n] = 0;} else s--;
	}

	for(i = ((int)databits-1); i >= 0; i--)
	{
        buffer[n] |= ((data >> i) & 0x1) << s;
        if(s == 0) {s = 7; n++; buffer[n] = 0;} else s--;
	}
	
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)buffer,
		.rx_buf = (unsigned long)NULL,
		.len = (GSPI_SPI_COMM_WRITE_BITS)/8 + 1,	// byte adjust
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	
	// Set Intended CS to 0
	if (-1 == GPIOWrite(fd_gpio, LOW))
		return(0);

	// Write and Read the data
	ret = ioctl(ftHandle, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
	
	// Set Intended CS to 1
	if (-1 == GPIOWrite(fd_gpio, HIGH))
		return(0);

	/* Dummy Bits, for issue aditional clk cycles */
	tr.len = 1;
	ret = ioctl(ftHandle, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
	

	return 1;
}

int read_single_word(uint32_t address, uint32_t sizeaddr, uint32_t* data, uint32_t databits)
{
	int ret;
	int i, n, s;

	/* CS_High + Write command + Address */
	buffer[0] = 0xBA;
	buffer[1] = 0xAB;
	buffer[2] = 0xAB;
	buffer[0] = GSPI_TASK_READ << 6;	/* Write command (2bit, 6-bit displaced)*/
	n = 0; s = 5;
	for(i = ((int)sizeaddr-1); i >= 0; i--)
	{
        buffer[n] |= ((address >> i) & 0x1) << s;
        if(s == 0) {s = 7; n++; buffer[n] = 0;} else s--;
	}

	for(i = ((int)databits-1); i >= 0; i--)
	{
        buffer[n] |= ((0 >> i) & 0x1) << s;
        if(s == 0) {s = 7; n++; buffer[n] = 0;} else s--;
	}
	
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)buffer,
		.rx_buf = (unsigned long)rbuffer,
		.len = (GSPI_SPI_COMM_READ_BITS+GSPI_SPI_READ_SEND_BITS)/bits + 1,	// byte adjust
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	
	// Set Intended CS to 0
	if (-1 == GPIOWrite(fd_gpio, LOW))
		return(0);

	// Write and Read the data
	ret = ioctl(ftHandle, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
	
	// Set Intended CS to 1
	if (-1 == GPIOWrite(fd_gpio, HIGH))
		return(0);

	n = 0; s = 7 - GSPI_SPI_COMM_READ_BITS + 1;
	while(s < 0) {s += 8; n++;}
	(*data) = 0;
	for(i = ((int)databits-1); i >= 0; i--)  /*IS NOT FUCKING LITTLE ENDIAN*/
	{
        (*data) |= ((rbuffer[n] >> s) & 0x1) << i;
        if(s == 0) {s = 7; n++;} else s--;
	}

	/* Dummy Bits, for issue aditional clk cycles */
	tr.len = 1;
	ret = ioctl(ftHandle, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	return 1;
}

int spi_init(void)
{
    strcpy(GSPI_NAME, "No Device");
    GSPI_DONE = 0;
		
	// ** Configuring GPIO
    
    // Enable GPIO
	if (-1 == GPIOExport(GPIO_CS))
		return(0);
 
	// Set GPIO direction
	if (-1 == GPIODirection(GPIO_CS, OUT))
		return(0);

	// Open file feeder for GPIO
	fd_gpio = GPIOOpenValue(GPIO_CS, O_WRONLY);
	if (-1 == fd_gpio)
		return(0);
	
	// Set Intended CS to 1
	if (-1 == GPIOWrite(fd_gpio, HIGH))
		return(0);
		
	// ** Finish Configuring GPIO
	
	// ** Configuring SPIDEV
	int ret;
	ftHandle = open(device, O_RDWR);
	if (ftHandle < 0)
		pabort("can't open device");

	// SPI mode
	mode = 0;
	//mode |= SPI_LOOP;
	//mode |= SPI_CPHA;
	//mode |= SPI_CPOL;
	//mode |= SPI_LSB_FIRST;
	//mode |= SPI_CS_HIGH;
	//mode |= SPI_3WIRE;
	//mode |= SPI_NO_CS;
	//mode |= SPI_READY;
	ret = ioctl(ftHandle, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(ftHandle, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	// bits per word
	ret = ioctl(ftHandle, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(ftHandle, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	// max speed hz
	ret = ioctl(ftHandle, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(ftHandle, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	//transfer(ftHandle);
	
	// ** Finish Configuring SPIDEV

    GSPI_DONE = 1;
    sprintf(GSPI_NAME, "SPI(%s) \n", device);
    return 1;
}

void spi_close(void)
{
	GSPI_DONE = 0;
    strcpy(GSPI_NAME, "No Device");
	
	close(ftHandle);
	GPIOCloseValue(fd_gpio);
}
