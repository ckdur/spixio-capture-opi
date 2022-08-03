#include <stdio.h>
#include <math.h>
#include "spixio.h"

// Compile with:
// gcc main.c spixio-opi.c -I./ -o spixio-capture -lm

// CKDUR: Radix you want to display (only valid 2, 8, 10 and 16)
#define RADIX 2

// CKDUR: Program will only capture this ammount of lectures
#define COUNT 10000

// Capture program for aamaya's chip
int get_ndig(int word, int radix)
{
    return (int)(log((double)((1<<word)-1))/log((double)radix)) + 1;
}

void sprint_radix(char* chbuf2, uint32_t nvalue, int ndig, int radix)
{
    int i;
    if(radix == 16) sprintf(chbuf2, "%.*x", ndig, nvalue);
    if(radix == 10) sprintf(chbuf2, "%u", (unsigned int)nvalue);
    if(radix == 8) sprintf(chbuf2, "%.*o", ndig, nvalue);
    if(radix == 2) {
        for(i = 0; i < ndig; i++)
        {
            chbuf2[i] = ((nvalue >> (ndig - i - 1)) & 0x1)?'1':'0';
        }
        chbuf2[i] = 0;
    }
}

int main(int argc, char *argv[])
{
	printf("Welcome to spix I/O handler!\n");
	printf("aamaya compilation, " __DATE__ ", " __TIME__ "\n");
	
	GSPI_INPUTS = 3;
	GSPI_OUTPUTS = 4;
	GSPI_WORD = 8;
    GSPI_MAXOBJ = FUNC_GSPI_MAXOBJ;
    GSPI_ADDR = FUNC_GSPI_SPI_ADDR_BITS;
	
	if(!spi_init()) 
	{
		fprintf(stderr, "Failed to init SPI (File: %s, Line %d)\n", __FILE__, __LINE__);
		return 1;
	}

	int waddr_size = FUNC_GSPI_SPI_WADDR_BITS;
	int raddr_size = FUNC_GSPI_SPI_RADDR_BITS;
	
	// CKDUR: Replace 0x00's for writting to Registers
	if(GSPI_DONE) write_single_word(0, waddr_size, 0x00, GSPI_WORD);	// Reg 1
	if(GSPI_DONE) write_single_word(1, waddr_size, 0x00, GSPI_WORD);	// Reg 2
	if(GSPI_DONE) write_single_word(2, waddr_size, 0x00, GSPI_WORD);	// Reg 3
	if(GSPI_DONE) write_single_word(3, waddr_size, 0x00, GSPI_WORD);	// Reg 4
	if(GSPI_DONE) write_single_word(0, waddr_size, 0xCC, GSPI_WORD);	// Reg 1
	if(GSPI_DONE) write_single_word(1, waddr_size, 0xC2, GSPI_WORD);	// Reg 2
	if(GSPI_DONE) write_single_word(2, waddr_size, 0x86, GSPI_WORD);	// Reg 3
	if(GSPI_DONE) write_single_word(3, waddr_size, 0x03, GSPI_WORD);	// Reg 4
	if(GSPI_DONE) write_single_word(0, waddr_size, 0xC4, GSPI_WORD);	// Reg 1
	
	int count = 0;
	while(1)
	{
		if(COUNT && count >= COUNT) break;
	
		uint32_t nvalue;
        char chbuf2[100];
		/*if(GSPI_DONE) read_single_word(0, raddr_size, &nvalue, GSPI_WORD);	// Reg 1
		sprint_radix(chbuf2, nvalue, get_ndig(GSPI_WORD, RADIX), RADIX);	// Print to string Reg 1
		fprintf(stdout, "%s", chbuf2);										// Print Reg 1 to stdout
		fprintf(stdout, ", ");												// Print a comma and space
		if(GSPI_DONE) read_single_word(1, raddr_size, &nvalue, GSPI_WORD);	// Reg 2
		sprint_radix(chbuf2, nvalue, get_ndig(GSPI_WORD, RADIX), RADIX);	// Print to string Reg 1
		fprintf(stdout, "%s", chbuf2);										// Print Reg 1 to stdout
		fprintf(stdout, ", ");												// Print a comma and space
		if(GSPI_DONE) read_single_word(2, raddr_size, &nvalue, GSPI_WORD);	// Reg 3
		sprint_radix(chbuf2, nvalue, get_ndig(GSPI_WORD, RADIX), RADIX);	// Print to string Reg 1
		fprintf(stdout, "%s", chbuf2);										// Print Reg 1 to stdout*/

		uint32_t reg1;
		if(GSPI_DONE) read_single_word(0, raddr_size, &reg1, GSPI_WORD);	// Reg 1
		uint32_t reg2;
		if(GSPI_DONE) read_single_word(1, raddr_size, &reg2, GSPI_WORD);	// Reg 2
		uint32_t reg3;
		if(GSPI_DONE) read_single_word(2, raddr_size, &reg3, GSPI_WORD);	// Reg 3

		// Print Comp 1
		fprintf(stdout, "%d, ", (int)((reg3 >> 6) & 0x3));
		
		// Print Comp 2
		fprintf(stdout, "%d, ", (int)((reg3 >> 4) & 0x3));

		// Print DAC 1
		fprintf(stdout, "%d, ", (int)( ((reg3 & 0xF) << 4) | ((reg2 >> 4) & 0xF) ) );

		// Print DAC 2
		fprintf(stdout, "%d", (int)( ((reg2 & 0xF) << 4) | ((reg1 >> 4) & 0xF) ) );
		
		fprintf(stdout, "\n");												// Print a new line
		
		count ++;
    }
    
    spi_close();
	
	return 0;
}

