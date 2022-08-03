#ifndef SPIXIO_H
#define SPIXIO_H

#include <inttypes.h>

extern uint32_t GSPI_INPUTS;
extern uint32_t GSPI_OUTPUTS;
extern uint32_t GSPI_WORD;
extern uint32_t GSPI_ADDR;
extern uint32_t GSPI_MAXOBJ;
extern uint8_t GSPI_DONE;
extern char GSPI_NAME[500];

/* Application specific macro definations */
#define CHANNEL_TO_OPEN				0	/*0 for first available channel, 1 for next... */
#define SPI_DEVICE_BUFFER_SIZE		256
#define SPI_CLOCK_RATE_HZ			50000000
//#define GSPI_INPUTS					6
//#define GSPI_OUTPUTS				8
//#define GSPI_WORD					8
#define FUNC_GSPI_MAXOBJ			(GSPI_OUTPUTS>GSPI_INPUTS?GSPI_OUTPUTS:GSPI_INPUTS);
#define GSPI_SPI_DUMMY_BITS			3
#define GSPI_SPI_TASK_BITS			2
#define FUNC_GSPI_SPI_ADDR_BITS 	clogb2(GSPI_MAXOBJ)
#define FUNC_GSPI_SPI_RADDR_BITS 	clogb2(GSPI_MAXOBJ)  // Because raddr bits in ugly version
#define FUNC_GSPI_SPI_WADDR_BITS 	clogb2(GSPI_OUTPUTS)  // Because waddr bits in ugly version
#define GSPI_SPI_DATA_BITS 			databits
#define GSPI_SPI_ADDR_BITS          sizeaddr
#define GSPI_SPI_COMM_WRITE_BITS	(GSPI_SPI_TASK_BITS+GSPI_SPI_ADDR_BITS+GSPI_SPI_DATA_BITS)
// Fix because capture is on OUTPUTS, only works properly if nbits_read = nbits_write+1
// NOTE: if difference between OUTPUTS and INPUTS its very high, maybe this doesnt work
#define GSPI_SPI_COMM_READ_BITS		((GSPI_SPI_TASK_BITS+GSPI_SPI_ADDR_BITS+GSPI_SPI_DATA_BITS)+(GSPI_OUTPUTS>=GSPI_INPUTS?1:0))
#define GSPI_SPI_READ_SEND_BITS		GSPI_SPI_DATA_BITS
#define GSPI_TASK_READ				0x1
#define GSPI_TASK_WRITE				0x2
#define GSPI_TASK_RW				0x3

int read_write_single_word(uint32_t address, uint32_t sizeaddr, uint32_t* data, uint32_t databits);
int write_single_word(uint32_t address, uint32_t sizeaddr, uint32_t data, uint32_t databits);
int read_single_word(uint32_t address, uint32_t sizeaddr, uint32_t* data, uint32_t databits);
int spi_init(void);
void spi_close(void);
int clogb2(int value);

#endif // SPIXIO_H
