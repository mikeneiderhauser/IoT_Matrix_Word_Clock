#ifndef WORD_CLOCK_CONFIG_H
#define WORD_CLOCK_CONFIG_H

#define WORD_CLOCK_EEPROM_WRITTEN 24
#include <stdint.h>
//#include <string.h>

#define NTP_SRV_LEN 32

static const char default_ntp_srv[33] = "pool.ntp.org";
static const int default_disp_on_hour = 5;
static const int default_disp_on_min = 0;
static const int default_disp_off_hour = 22; // TBD Military hours or AM vs PM
static const int default_disp_off_min = 30;
static const int default_disp_brightness = 255;
static const int32_t default_tmz = -5;

struct clock_config {
    uint8_t magic_byte;
    char ntp_srv[33];
    int disp_on_hour;
    int disp_on_min;
    int disp_off_hour;
    int disp_off_min;
    int disp_brightness;  // TBD
    int32_t tmz;
};

struct clock_time {
    uint8_t hour;   // 0-23 hour format
    uint8_t min;    // 0-60
    uint8_t sec;    // 0-60
    uint8_t month;  // 1-12
    uint8_t day;    // 1-31
    uint16_t year;  // 2015-2114
};

#endif
