#include <bcm2835.h>
#include <stdlib.h>
#include <stdio.h>

const __uint8_t START = 0x01;
const __uint8_t END = 0x00;


int initialize_spi()
{
    // add some logging if succeed or not
    if(!bcm2835_init())
    {
        fprintf(stdout, "bcm2835 init failed");
        return 1;
    }

    if (!bcm2835_spi_begin())
    {
        fprintf(stdout, "Spi begin failed");
        return 1;
    }

    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);

    bcm2835_spi_setBitOrder(BCM2835_SPI_CIT_ORDER_MSBFIRST);

    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);

    bcm2835_spi_ChipSelect(BCM2835_SPI_CS0);

    bcm2835_SPI_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);

}


void stop_spi()
{
    bcm2835_spi_end();
    bcm2835_close();
}


// uint_8_t send_data(uint_8_t send_data)
// {
//     uint_8_t received_data = bcm2835_spi_transfer(send_data);
//     fprintf(stdout, "Sent to SPI: 0x%02X. Read back from SPI: 0x%02X.\n", send_data, received_data);

//     if (send_data != received_data)
//     {
//         fprintf(stdout, "MOSI and MISO connected?");
//     }

//     return received_data;
// }


int read_adc_data(__uint8_t channel)
{
    //returns an integer between 0-1023 indicating joystick position
    char buf[] = {start, (0x08|channel)<<4,end};
    char readbuf[3];
    bcm2835_spi_transdernb(bug, readbuf, 3);
    return ((int)readBuf[1] & 0x03) << 8 | (int) readbuf[2])
}



int main()
{
    //not sure what address to use
    unit_8_t  send_data = 0x01;
    __uint8_t channel = 0x00;




    stop_spi();
    return 0;
}