/*

Hardware
  Tested on ATMega328P
  Should work on ATMega8
  Should work on ATTiny4313
  Uses < 128 Bytes RAM
  Uses <  2K Bytes Flash
  
Matrix Layout

Rows are common cathode
Column By Row (CC, ..N.., RR)

    Base Byte + 1  (MSB)      Base Byte (LSB)            Base  Base 
CC,CC,CC,CC,CC,CC,CC,CC,   CC,CC,CC,CC,CC,CC,CC,CC   RR  BIN   Byte
---------------------------------------------------------------------
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  15  240   30
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  14  224   28
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  13  208   26
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  12  192   24
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  11  176   22
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  10  160   20
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  09  144   18
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  08  128   16
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  07  112   14
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  06  096   12
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  05  080   10
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  04  064   08
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  03  048   06
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  02  032   04
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  01  016   02
15,14,13,12,11,10,09,08,   07,06,05,04,03,02,01,00,  00  000   00
*/

#include <SPI.h>

#define ANODE_DAT 8  //PB0
#define ANODE_CLK A0 //PC0 (ADC0)
#define ANODE_LAT 9  //PB1
#define ANODE_RST A1 //PC1 (ADC1)

#define CATHODE_DAT 3  //PD3
#define CATHODE_CLK A3 //PC3 (ADC3)
#define CATHODE_LAT A2 //PC2 (ADC2)
#define CATHODE_RST 2  //PD2

#define SPI_MISO  12
#define SPI_MOSI  11
#define SPI_CLK   13
#define SPI_CS    10

#define LED_ON_TIME 50
#define LED_OFF_TIME 50

#define DEBUG
#ifdef DEBUG
    #include <string.h>
    #ifdef LED_ON_TIME
        #undef LED_ON_TIME
    #endif
    #define LED_ON_TIME 250

    #ifdef LED_OFF_TIME
        #undef LED_OFF_TIME
    #endif
    #define LED_OFF_TIME 250
#endif

uint16_t temp;
uint16_t buf[2][16]; // buffer similar to buffer used in matrix driver  32bytes * 2 = 64 bytes RAM
uint8_t cur_buf_idx = 0;
uint8_t cur_buf_idx_spi = 1;

uint8_t buf_swap = 0;

uint8_t spi_byte_counter = 0;

uint8_t anode_count = 0;
uint8_t cathode_count = 0;
uint16_t anode_dat = 0;
uint16_t cathode_dat = 0;

void shiftout_16(uint8_t data_pin, uint8_t clk_pin, uint8_t lat_pin, uint8_t rst_pin, uint16_t data) {
    shiftOut(data_pin, clk_pin, MSBFIRST, (uint8_t)(0xff&(data >> 8)));
    shiftOut(data_pin, clk_pin, MSBFIRST, (uint8_t)(data&0xff));
    digitalWrite(lat_pin, HIGH);
    digitalWrite(lat_pin, LOW);
}

void setup() {
    // Setup anode pins
    pinMode(ANODE_DAT, OUTPUT);
    pinMode(ANODE_CLK, OUTPUT);
    pinMode(ANODE_LAT, OUTPUT);
    pinMode(ANODE_RST, OUTPUT);

    // Write default values to pins
    digitalWrite(ANODE_DAT, LOW);
    digitalWrite(ANODE_CLK, LOW);
    digitalWrite(ANODE_LAT, LOW);
    digitalWrite(ANODE_RST, LOW);
    digitalWrite(ANODE_RST, HIGH);  // Active Low - take 595 out of reset

    // Setup cathode pins
    pinMode(CATHODE_DAT, OUTPUT);
    pinMode(CATHODE_CLK, OUTPUT);
    pinMode(CATHODE_LAT, OUTPUT);
    pinMode(CATHODE_RST, OUTPUT);

    // Write default values to pins
    digitalWrite(CATHODE_DAT, LOW);
    digitalWrite(CATHODE_CLK, LOW);
    digitalWrite(CATHODE_LAT, LOW);
    digitalWrite(CATHODE_RST, LOW);
    digitalWrite(CATHODE_RST, HIGH);  // Active Low - take 595 out of reset

    #ifdef DEBUG
    // Initial values for debugging only
    memset(&buf[0][0], 0xffff, 16);
    memset(&buf[1][0], 0xffff, 16);
    #endif

    // Setup SPI for slave mode
    pinMode(SPI_MISO, OUTPUT);
    SPCR | _BV(SPE);  // Set SPI as Slave
    SPI.attachInterrupt(); // Enable SPI Data interrupt
}

void loop() {    
    // Turn matrix off / matrix clear
    shiftout_16(CATHODE_DAT, CATHODE_CLK, CATHODE_LAT, CATHODE_RST, 0);
    shiftout_16(ANODE_DAT, ANODE_CLK, ANODE_LAT, ANODE_RST, 0);

    if(buf_swap == 1) {
        buf_swap = 0;
        spi_byte_counter = 0;
        if(cur_buf_idx == 0)
        {
            cur_buf_idx = 1;
            cur_buf_idx_spi = 0;
        }
        else
        {
            cur_buf_idx = 0;
            cur_buf_idx_spi = 1;
        }
    }
    
    // activate row (cathode)
    cathode_dat = (1 << cathode_count);
    shiftout_16(CATHODE_DAT, CATHODE_CLK, CATHODE_LAT, CATHODE_RST, cathode_dat);

    // activate each col
    for(anode_count = 0; anode_count < 16; anode_count++) {
        // for each anode
        anode_dat = buf[cur_buf_idx][cathode_count] & (1 << anode_count);

        // Turn anode on
        shiftout_16(ANODE_DAT, ANODE_CLK, ANODE_LAT, ANODE_RST, anode_dat);
        #if LED_ON_TIME > 0
        delay(LED_ON_TIME);
        #endif

        // Turn anode off
        shiftout_16(ANODE_DAT, ANODE_CLK, ANODE_LAT, ANODE_RST, 0);
        #if LED_OFF_TIME > 0
        delay(LED_OFF_TIME);
        #endif
    }

    (cathode_count == 15) ? cathode_count = 0 : cathode_count++;
}

// SPI interrupt handler
ISR (SPI_STC_vect)
{
    // spi_byte_counter / 2; -> truncated index into buffer
    // spi_byte_counter % 2; -> 0 indicates lower, 1 inducates upper of uint16_t

    if (spi_byte_counter < 32)
    {
        temp = ((uint16_t)SPDR) & 0xff;
        if((spi_byte_counter % 2) == 0)
        {
            buf[cur_buf_idx_spi][(spi_byte_counter / 2)] = temp;  // also clears top byte
        }
        else
        {
            buf[cur_buf_idx_spi][(spi_byte_counter / 2)] |= temp << 8;
        }
        
        spi_byte_counter++;
        if (spi_byte_counter >= 32)
        {
            buf_swap = 1;
            digitalWrite(MISO, HIGH);
        }
    }
    else
    {
        temp = SPDR; // eat SPDR
    }
}
