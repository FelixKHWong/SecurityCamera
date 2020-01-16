#include <bcm2835.h>
#include <stdlib.h>
#include <stdio.h>

#define MAX_LEN 32

int initialize_i2c()
{
    if(!bcm2835_init())
    {
        fprintf(stdout, "bcm2835 init failed");
        return 1;
    }
    if (!bcm2835_i2c_begin())
    {
        fprintf(stdout, "I2c begin failed");
        return 1;
    }

    bcm2835_i2c_setSlaveAddress(slave_address);
    bcm2835_i2c_setClockDivider(clk_div);
    fprintf(stderr, "Clock divider set to: %d\n", clk_div);
    fprintf(stderr, "len set to: %d\n", len);
    fprintf(stderr, "Slave address set to: %d\n", slave_address);   
}

void stop_i2c()
{
    bcm2835_i2c_end();
    bcm2835_close();
}

int read_slave(int buf_len)
{
    //need to specify length of buffer data first
    char buf[MAX_LEN];
    for (int i=0; i<MAX_LEN; i++)
    {
        buf[i] = 'n';
        data = bcm2835_i2c_read(buf, buf_len);
    }


}

int write_slave()
{   char write_buf[MAX_LEN];
    bcm2835_i2c_write(write_buf, len(write_buf));
}

int main()
{


}