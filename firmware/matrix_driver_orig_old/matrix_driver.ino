#include <SPI.h>
#include <Wire.h>

/*  
 * 16x16 LED Matrix Driver
 *   Using ATMEGA8
 * 
 * Features:
 *   - SPI, I2C, and UART Hardware support
 *   - Double-buffered display 
 *   - Port expansion via 74HC595 Shift Registers
 *   - Row LEDs driven (anode) via 74HC595 Shift registers
 *   - Col LEDs driven (cathod) via 74HC595 Shift registers and ULN23008 Darlington arrays
 *   
Matrix Layout

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

//#define ALL_ANODES  // Define to allow 595's to drive all 16 anodes

// Pin Defines
#define IO_595_ROW_DAT 8
#define IO_595_ROW_CLK A0
#define IO_595_ROW_LAT 9
#define IO_595_ROW_RST A1

#define IO_595_COL_DAT 3
#define IO_595_COL_CLK A3
#define IO_595_COL_LAT A2
#define IO_595_COL_RST 2

#define SPI_MISO  12
#define SPI_MOSI  11
#define SPI_CLK   13
#define SPI_CS    10

//#define SDA A4
//#define SCL A5

#define SEL_2 5
#define SEL_1 6
#define SEL_0 7

// Constant Defines
#define I2C_SLV_ADDR_BASE 0x30
#define DEFAULT_BAUD 115200
#define SERIAL_BUFFER_FULL 0xF0
#define SERIAL_BUFFER_READY 0xA0

uint8_t buf[2][32];      // double buffer based on index
uint8_t cur_buf_idx;     // buffer used for multiplex loop
uint8_t input_buf_idx;   // buffer used for input transer (SPI, I2C, and UART)

uint8_t row_num;         // current row of multiplex loop
uint8_t row_data_lsb;    // data for lsb row shift register
uint8_t row_data_msb;    // data for msb row shift register

uint8_t base_idx;        // base index for row in multiplex
uint8_t input_byte_ct;   // counter for input data (SPI, I2C, and UART)
uint8_t swap_buf_flag;   // flag used to indicate when to swap buffers
uint8_t i2c_addr;        // slave I2C address

uint8_t one_hot_ctr;
uint8_t one_hot_mask[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

void setup() {
    // Setup Row Pins
    pinMode(IO_595_ROW_DAT, OUTPUT);
    pinMode(IO_595_ROW_CLK, OUTPUT);
    pinMode(IO_595_ROW_LAT, OUTPUT);
    pinMode(IO_595_ROW_RST, OUTPUT);

    // Write default values to pins
    digitalWrite(IO_595_ROW_DAT, LOW);
    digitalWrite(IO_595_ROW_CLK, LOW);
    digitalWrite(IO_595_ROW_LAT, LOW);

    // Reset Shift Register (does not transfer to storage register)
    digitalWrite(IO_595_ROW_RST, LOW);  // Active Low (Perform Reset to shift register)
    digitalWrite(IO_595_ROW_LAT, HIGH); // Latch to transfer shift register to storage register
    digitalWrite(IO_595_ROW_LAT, LOW);
    digitalWrite(IO_595_ROW_RST, HIGH);  // Active Low
      
    // Setup Col Pins
    pinMode(IO_595_COL_DAT, OUTPUT);
    pinMode(IO_595_COL_CLK, OUTPUT);
    pinMode(IO_595_COL_LAT, OUTPUT);
    pinMode(IO_595_COL_RST, OUTPUT);
    
    // Write default values to pins
    digitalWrite(IO_595_COL_DAT, LOW);
    digitalWrite(IO_595_COL_CLK, LOW);
    digitalWrite(IO_595_COL_LAT, LOW);
  
    // Reset Shift Register (does not transfer to storage register)
    digitalWrite(IO_595_COL_RST, LOW);  // Active Low (Perform Reset to shift register)
    digitalWrite(IO_595_COL_LAT, HIGH); // Latch to transfer shift register to storage register
    digitalWrite(IO_595_COL_LAT, LOW);
    digitalWrite(IO_595_COL_RST, HIGH);  // Active Low

    pinMode(SEL_2, INPUT_PULLUP);
    pinMode(SEL_1, INPUT_PULLUP);
    pinMode(SEL_0, INPUT_PULLUP);

    i2c_addr = I2C_SLV_ADDR_BASE + (~digitalRead(SEL_0) & 1);
    i2c_addr += ((~digitalRead(SEL_1) & 1) << 1);
    i2c_addr += ((~digitalRead(SEL_2) & 1) << 2);

    cur_buf_idx = 0;
    input_buf_idx = 1;
    row_num = 0;
    base_idx = 0;
    input_byte_ct = 0;
    swap_buf_flag = 0;

    // Setup SPI for slave mode
    pinMode(SPI_MISO, OUTPUT);
    SPCR | _BV(SPE);  // Set SPI as Slave
    SPI.attachInterrupt(); // Enable SPI Data interrupt

    // Setup I2C for slave mode
    Wire.begin(i2c_addr);
    Wire.onReceive(i2c_rx_handler);

    // Setup UART
    Serial.begin(DEFAULT_BAUD);
    Serial.println(i2c_addr);
}

void loop() {
    // check for buffer swap
    if(swap_buf_flag == 1)
    {
      // swap buffer idx's
      if(cur_buf_idx == 0)
      {
        cur_buf_idx = 1;
        input_buf_idx = 0;
      }
      else
      {
        cur_buf_idx = 0;
        input_buf_idx = 1;
      }
      // clear flag
      swap_buf_flag = 0;
      // handle spi buf ready
      digitalWrite(MISO, LOW);
      // handle serial buf ready
      Serial.write(SERIAL_BUFFER_READY);
    }
    
    // reset row shift register and storage register
    digitalWrite(IO_595_ROW_RST, LOW); 
    digitalWrite(IO_595_ROW_LAT, HIGH);
    digitalWrite(IO_595_ROW_LAT, LOW);
    digitalWrite(IO_595_ROW_RST, HIGH);

    // activate proper column (1-hot)
    if(row_num == 0)
    {
      digitalWrite(IO_595_COL_DAT, HIGH);
    }
    else
    {
      digitalWrite(IO_595_COL_DAT, LOW);
    }

    // Clock in col bit
    digitalWrite(IO_595_COL_CLK, HIGH);
    digitalWrite(IO_595_COL_CLK, LOW);

    // Latch col storage register
    digitalWrite(IO_595_COL_LAT, HIGH);
    digitalWrite(IO_595_COL_LAT, LOW);
  
    // calculate offset
    base_idx = row_num * 2;

    // get row data
    row_data_lsb = buf[cur_buf_idx][base_idx];
    row_data_msb = buf[cur_buf_idx][base_idx + 1];

    #ifdef ALL_ANODES
    // write all row data to shift register
    shiftOut(IO_595_ROW_DAT, IO_595_ROW_CLK, MSBFIRST, row_data_msb);
    shiftOut(IO_595_ROW_DAT, IO_595_ROW_CLK, MSBFIRST, row_data_lsb);
    // Latch row storage register
    digitalWrite(IO_595_ROW_LAT, HIGH);
    digitalWrite(IO_595_ROW_LAT, LOW);
    #else
    // write all row data bitwise to shift register
    // MSB
    for(one_hot_ctr = 8; one_hot_ctr > 0; one_hot_ctr--)
    {
        shiftOut(IO_595_ROW_DAT, IO_595_ROW_CLK, MSBFIRST, 
            (row_data_msb & one_hot_mask[one_hot_ctr-1]));
        shiftOut(IO_595_ROW_DAT, IO_595_ROW_CLK, MSBFIRST, 0);
        // Latch row storage register
        digitalWrite(IO_595_ROW_LAT, HIGH);
        digitalWrite(IO_595_ROW_LAT, LOW);
    }

    // LSB
    for(one_hot_ctr = 8; one_hot_ctr > 0; one_hot_ctr--)
    {
        shiftOut(IO_595_ROW_DAT, IO_595_ROW_CLK, MSBFIRST, 0);
        shiftOut(IO_595_ROW_DAT, IO_595_ROW_CLK, MSBFIRST, 
            (row_data_lsb & one_hot_mask[one_hot_ctr-1]));
        // Latch row storage register
        digitalWrite(IO_595_ROW_LAT, HIGH);
        digitalWrite(IO_595_ROW_LAT, LOW);
    }
    #endif

    // Increment row number
    row_num = (row_num == 16) ? 0 : ++row_num;
}

void add_byte_to_buf(uint8_t data)
{
    if(input_byte_ct <= 31)
    {
        // add byte to double buffer
        buf[input_buf_idx][input_byte_ct] = data;
        // Increment SPI byte counter
        input_byte_ct++;
        if(input_byte_ct > 31)
        {
            // set swap flag
            swap_buf_flag = 1;
        
            // handle SPI buf full
            // set MISO error vlaue to HIGH (in the event we get more spi data)
            digitalWrite(MISO, HIGH);
            // handle serial buf full
            Serial.write(SERIAL_BUFFER_FULL);
        }
    }
}

// SPI interrupt handler
ISR (SPI_STC_vect)
{
    add_byte_to_buf(SPDR);
}

// I2C interrupt handler
void i2c_rx_handler(int numBytes)
{
    for(uint8_t i2c_ctr = 0; i2c_ctr < numBytes; i2c_ctr++)
    {
        add_byte_to_buf(Wire.read());
    }
}

// UART interrupt handler (not a true interrupt)
void serialEvent()
{
    // typical 32 byte transfer
    while(Serial.available())
    {
        add_byte_to_buf(Serial.read());
    }
}

