#include "clock_config.h"
#include "bitmaps.h"

#include "SparkTime.h"
#include "SparkIntervalTimer.h"
#include <string.h>

//#define ENABLE_DST_SWITCH
#define DST_SWITCH 10

uint8_t display[BITMAP_SZ];
uint8_t written_display[BITMAP_SZ];

// RTC vars
UDP UDPClient;
SparkTime rtc;
unsigned long currentTime;
unsigned long lastTime = 0UL;
String timeStr;

// Uncomment below to set resolution of NTP update to mins vs secs
//#define NTP_TIME_UPDATE_INTERVAL_MIN 1

#ifdef NTP_TIME_UPDATE_INTERVAL_MIN
#define NTP_TIME_UPDATE_INTERVAL_SEC (NTP_TIME_UPDATE_INTERVAL_MIN * 60)
#else
#define NTP_TIME_UPDATE_INTERVAL_SEC 1
#endif

// Event trigger flags
volatile uint8_t update_time = 0;
int update_disp = 0;
int enable_disp = 0;
volatile uint8_t trigger_reboot = 0;

clock_config settings;  // global settings

clock_time ntp_time; // shared simplified time & date struct

IntervalTimer ntp_update_timer;

/* ----------------- START EEPROM FUNCTIONS ----------------- */
uint8_t read_settings(clock_config *c)
{
    clock_config tmp; // Temp storage for reading from eeprom
    EEPROM.get(0, tmp); // read from eeprom
    tmp.ntp_srv[32] = 0;
    if(tmp.magic_byte != WORD_CLOCK_EEPROM_WRITTEN)
    {
        // SET DEFAULTS
        memcpy(&tmp.ntp_srv[0], &default_ntp_srv[0], NTP_SRV_LEN);
        tmp.disp_on_hour = default_disp_on_hour;
        tmp.disp_on_min = default_disp_on_min;
        tmp.disp_off_hour = default_disp_off_hour;
        tmp.disp_off_min = default_disp_off_min;
        tmp.disp_brightness = default_disp_brightness;
        tmp.tmz = default_tmz;
        // Write EEPROM
        write_settings(&tmp);
    }
    memcpy(c, &tmp, sizeof(clock_config)); //copy to global settings from temp
    return 0;
}

uint8_t write_settings(clock_config *c)
{
    c->ntp_srv[32] = 0;
    c->magic_byte = WORD_CLOCK_EEPROM_WRITTEN;
    EEPROM.put(0, *c); // write to eeprom
    return 0;
}

// Clears EEPROM Settings
uint8_t clear_settings(clock_config *c)
{
    memset(c, 0, sizeof(clock_config));
    EEPROM.put(0, *c);
    return 0;
}

// Clears EEPROM and reboots device
int clear_settings_helper(String cmd)
{
    clear_settings(&settings);
    trigger_reboot = 1;
    return 0;
}

/* ----------------- END EEPROM FUNCTIONS ----------------- */


/* ----------------- START DISPLAY FUNCTIONS ----------------- */
// Clear the display buffer (set all pixels to off state)
uint8_t clear_display_buffer(uint8_t * d_buf, uint8_t sz)
{
    set_display_buffer(d_buf, sz, BM_OFF);
    return 0;
}

// Write the display buffer to display driver if different than last written buffer
// d_buf -> current working buffer
// c_buf -> last written buffer (compare buffer)
// sz -> size of buffer(s)
uint8_t write_display(uint8_t * d_buf, uint8_t * c_buf, uint8_t sz)
{
    uint8_t tmp; // to not overwrite d_buf
    if(memcmp(d_buf, c_buf, sz) != 0)
    {
        Serial.println("Writing to the display");
        // buffers are different.. write to driver
        for(uint8_t i = 0; i < BITMAP_SZ; i++)
        {
            tmp = d_buf[i];
            SPI.transfer(i);
        }
        memcpy(c_buf, d_buf, sz);
    }

    return 0;
}

// Set single bitmap to display buffer (used to clear)
uint8_t set_display_buffer(uint8_t * d_buf, uint8_t sz, uint8_t bm_idx)
{
    if (bm_idx >= BM_MAX) return 1;
    memcpy(d_buf, &(bitmaps[bm_idx][0]), sz);
    return 0;
}

// Bitwise (OR) add bitmap to display buffer
uint8_t add_bm_to_d_buf(uint8_t * d_buf, uint8_t sz, uint8_t bm_idx)
{
    Serial.print("Add bitmap: ");
    Serial.println(bm_idx);
    if (bm_idx >= BM_MAX) return 1;

    for(uint8_t i = 0; i < sz; i++)
    {
        d_buf[i] |= bitmaps[bm_idx][i];
    }
    return 0;
}

uint8_t calculate_bitmap(uint8_t * d_buf, uint8_t sz, clock_time * time)//uint8_t hr, uint8_t min, uint8_t mon, uint8_t day, uint16_t yr)
{
    uint8_t hr = time->hour;
    // adjust military time
    if (hr > 12){
        hr = hr - 12;
    }

    /*
    #ifdef DEBUG_MSG
    Serial.println("Calculating new display buffer");
    Serial.print("    Hour: "); Serial.println(hr);
    Serial.print("    Min : "); Serial.println(time->min);
    Serial.print("    Sec : "); Serial.println(time->sec);
    Serial.print("    Mon : "); Serial.println(time->month);
    Serial.print("    Day : "); Serial.println(time->day);
    Serial.print("    Year: "); Serial.println(time->year);
    #endif
    */
    clear_display_buffer(d_buf, sz);

    // Time
    if (hr == 12){ hr = 0; } // adjust hour offset
    // min
    if(time->min < 5) { // 00 // NO BITMAP
    }
    else if (time->min < 10) { add_bm_to_d_buf(d_buf, sz, BM_MINS_5); }
    else if (time->min < 15) { add_bm_to_d_buf(d_buf, sz, BM_MINS_10); }
    else if (time->min < 20) { add_bm_to_d_buf(d_buf, sz, BM_MINS_15); }
    else if (time->min < 25) { add_bm_to_d_buf(d_buf, sz, BM_MINS_20); }
    else if (time->min < 30) { add_bm_to_d_buf(d_buf, sz, BM_MINS_25); }
    else if (time->min < 35) { add_bm_to_d_buf(d_buf, sz, BM_MINS_30); }
    else if (time->min < 40) { add_bm_to_d_buf(d_buf, sz, BM_MINS_35); hr++; }
    else if (time->min < 45) { add_bm_to_d_buf(d_buf, sz, BM_MINS_40); hr++; }
    else if (time->min < 50) { add_bm_to_d_buf(d_buf, sz, BM_MINS_45); hr++; }
    else if (time->min < 55) { add_bm_to_d_buf(d_buf, sz, BM_MINS_50); hr++; }
    else { add_bm_to_d_buf(d_buf, sz, BM_MINS_55); hr++; }

    // hr
    if (hr == 12){ hr = 0; } // adjust hour offset (again incase of hour shift after 30 mins)
    // making an assumption about the hour index offset
    add_bm_to_d_buf(d_buf, sz, hr);

    // Date
    if (time->month == 1 && time->day == 1) { add_bm_to_d_buf(d_buf, sz, BM_NEW_YEAR); }
    if (time->month == 2 && time->day == 14) { add_bm_to_d_buf(d_buf, sz, BM_VALENTINES_DAY); }
    if (time->month == 3 && time->day == 17) { add_bm_to_d_buf(d_buf, sz, BM_ST_PATRICKS_DAY); }
    if (time->month == 7 && time->day == 4) { add_bm_to_d_buf(d_buf, sz, BM_FOURTH_JULY); }
    if (time->month == 10 && time->day == 31) { add_bm_to_d_buf(d_buf, sz, BM_HALLOWEEN); }
    if (time->month == 12 && time->day == 25) { add_bm_to_d_buf(d_buf, sz, BM_CHRISTMAS); }
    if (time->month == 8 && time->day == 15) { add_bm_to_d_buf(d_buf, sz, BM_CLOCK_DAY); }

    if (time->month == 7 && time->day == 24) { add_bm_to_d_buf(d_buf, sz, BM_BDAY_KELLY); }
    if (time->month == 6 && time->day == 20) { add_bm_to_d_buf(d_buf, sz, BM_BDAY_GIZMO); }
    if (time->month == 2 && time->day == 24) { add_bm_to_d_buf(d_buf, sz, BM_BDAY_GUS); }

    // Dynamic dates based on static lookup tables from year 2015 to year 2114
    if (time->month == 3 || time->month == 4)
    {
        // easter table lookup
        if (time->month == easter_mon_lookup_table[time->year-WORD_CLOCK_BASE_YEAR] &&
            time->day == easter_day_lookup_table[time->year-WORD_CLOCK_BASE_YEAR])
        {
            add_bm_to_d_buf(d_buf, sz, BM_EASTER);
        }
    }

    if (time->month == 5)
    {
        // mothers day table lookup
        if (time->day == mothers_day_lookup_table[time->year-WORD_CLOCK_BASE_YEAR])
        {
            add_bm_to_d_buf(d_buf, sz, BM_MOTHERS_DAY);
        }
    }
    if (time->month == 6)
    {
        // fathers day table lookup
        if (time->day == fathers_day_lookup_table[time->year-WORD_CLOCK_BASE_YEAR])
        {
            add_bm_to_d_buf(d_buf, sz, BM_FATHERS_DAY);
        }
    }

    if (time->month == 11)
    {
        // thanksgiving table lookup
        if (time->day == thanksgiving_lookup_table[time->year-WORD_CLOCK_BASE_YEAR])
        {
            add_bm_to_d_buf(d_buf, sz, BM_THANKSGIVING);
        }
    }
}
/* ----------------- END DISPLAY FUNCTIONS ----------------- */

/* ----------------- START TIME/RTC/NTP FUNCTIONS ----------------- */
void ntp_caller(clock_time * ntp_time)
{
    Serial.println("ntp_caller");

    currentTime = rtc.now();
    if (currentTime != lastTime) {
        ntp_time->sec = rtc.second(currentTime);
        ntp_time->hour = rtc.hour(currentTime);
        ntp_time->min = rtc.minute(currentTime);
        ntp_time->month = rtc.month(currentTime);
        ntp_time->day = rtc.day(currentTime);
        ntp_time->year = (uint16_t)rtc.year(currentTime);

        Serial.println(rtc.ISODateString(currentTime));
        lastTime = currentTime;
    }
} // end ntp Caller
/* ----------------- END TIME/RTC/NTP FUNCTIONS ----------------- */

// Timer helper function
void trigger_ntp_update(void)
{
    update_time = 1;
}

int config_wifi(String cmd)
{
    Serial.println("Configure Wifi");
    Serial.println(cmd);
    char toparse[70];

    char ssid[70];
    char password[70];
    uint8_t sec_type;

    strcpy(toparse, cmd.c_str());

    char *pch;
    uint8_t ctr = 0;
    pch = strtok(toparse,",");
    while (pch != NULL)
    {
        Serial.println(ctr);
        if(ctr == 0)
        {
            // ssid
            strcpy(ssid, pch);
        }
        else if(ctr == 1)
        {
            // pwd
            strcpy(password, pch);
        }
        else if(ctr == 2)
        {
            // sec type
            sec_type = atoi(pch);
        }
        else
        {
            //error
        }

        ctr++;
        pch = strtok(NULL,",");
    }

    if(sec_type == 3)
    {
        // Unsecured
        WiFi.setCredentials(ssid);
    }
    else if (sec_type == 2) {
        // WEP
        WiFi.setCredentials(ssid, password, WEP);
    }
    else if (sec_type == 1) {
        // WPA
        WiFi.setCredentials(ssid, password, WPA);
    }
    else if (sec_type == 0) {
        // WPA2
        WiFi.setCredentials(ssid, password, WPA2);
    }

    return 0;

}

int word_clock_configure(String cmd)
{
    Serial.println(cmd);
    //pool.ntp.org,6,15,22,45,-5,100
    char toparse[70];
    strcpy(toparse, cmd.c_str());

    char* pch;
    uint8_t ctr = 0;
    pch = strtok(toparse,",:");
    while (pch != NULL)
    {
        Serial.println(ctr);
        if(ctr == 0)
        {
            // ntp_srv
            strcpy(settings.ntp_srv, pch);
        }
        else if(ctr == 1)
        {
            // on_hour
            settings.disp_on_hour = atoi(pch);
        }
        else if(ctr == 2)
        {
            // on_min
            settings.disp_on_min = atoi(pch);
        }
        else if(ctr == 3)
        {
            // off_hour
            settings.disp_off_hour = atoi(pch);

        }
        else if(ctr == 4)
        {
            // off_min
            settings.disp_off_min = atoi(pch);
        }
        else if(ctr == 5)
        {
            // timezone
            settings.tmz = atoi(pch);
        }
        else if(ctr == 6)
        {
            // brightness
            settings.disp_brightness = atoi(pch);
            break;
        }
        ctr++;
        pch = strtok(NULL,",:");
    }

    write_settings(&settings);
    return 0;
}

int word_clock_reboot(String cmd)
{
    trigger_reboot = 1;
    return 0;
}

void setup() {
    // Init Serial
    Serial.begin(115200);

    // Read settings
    read_settings(&settings);

    Serial.println("Registering cloud functions and variables");

    // Register cloud functions and variables
    Particle.function("config", word_clock_configure);   // Function to post config to device (app config)
    Particle.function("config_wifi", config_wifi);
    Particle.function("set_defaults", clear_settings_helper);  // Function to clear settings from eeprom
    Particle.function("reboot", word_clock_reboot);

    if(Particle.variable("ntp_srv", settings.ntp_srv, STRING) == false)
    {
        Serial.println("Var: ntp_srv not registered");
    }
    if(Particle.variable("on_hour", settings.disp_on_hour) == false)
    {
        Serial.println("Var: disp_on_hour not registered");
    }
    if(Particle.variable("on_min", settings.disp_on_min) == false)
    {
        Serial.println("Var: disp_on_min not registered");
    }
    if(Particle.variable("off_hour", settings.disp_off_hour) == false)
    {
        Serial.println("Var: disp_off_hour not registered");
    }
    if(Particle.variable("off_min", settings.disp_off_min) == false)
    {
        Serial.println("Var: disp_off_min not registered");
    }
    if(Particle.variable("timezone", settings.tmz) == false)
    {
        Serial.println("Var: timezone not registered");
    }
    if(Particle.variable("brightness", settings.disp_brightness) == false)
    {
        Serial.println("Var: brightness not registered");
    }
    if(Particle.variable("enable_disp", enable_disp) == false)
    {
        Serial.println("Var: enable_disp not registered");
    }



    delay(5000); // debugging delay.. to open serial port
    Serial.println("Configure I/O");
    // Init IO
    #ifdef ENABLE_DST_SWITCH
    // DST indicator
    Serial.println("Configuring DST_SWITCH");
    #endif

    // Do outside ifdef for hardware stability
    pinMode(DST_SWITCH, INPUT_PULLUP);

    Serial.println("Configure SPI");
    // Init SPI
    SPI.begin();
    SPI.setClockDividerReference(SPI_CLK_ARDUINO);
    //SPI.setClockDivider(SPI_CLK_DIV4);
    SPI.setBitOrder(LSBFIRST);

    Serial.println("Configure display");
    // Init display buffers
    clear_display_buffer(display, BITMAP_SZ);
    clear_display_buffer(written_display, BITMAP_SZ);
    written_display[0] = 255; // force inconsistant buffers
    write_display(display, written_display, BITMAP_SZ); // writes over spi

    // Read settings
    read_settings(&settings);

    Serial.println("Starting the RTC");
    // Start NTP RTC
    rtc.begin(&UDPClient, settings.ntp_srv);
    rtc.setTimeZone(settings.tmz); // est offset

    // Set time defaults
    ntp_time.hour = 12;
    ntp_time.min = 0;
    ntp_time.sec = 255; // force loop below
    ntp_time.month = 1;  // January
    ntp_time.day = 1;   // first
    ntp_time.year = WORD_CLOCK_BASE_YEAR;

    Serial.println("Performing Clock Zero Sync");
    // zero sync.. can take up to 1 min
    while(ntp_time.sec != 0)
    {
        ntp_caller(&ntp_time);
        delay(500); // half second break
    }

    Serial.print("Configuring ntp update timer for ");
    Serial.print(NTP_TIME_UPDATE_INTERVAL_SEC);
    Serial.println(" seconds.");
    ntp_update_timer.begin(trigger_ntp_update,
        NTP_TIME_UPDATE_INTERVAL_SEC * 1000 * 2 , hmSec);

    Serial.println("Setup complete");
}

void loop()
{
    if(update_time == 1)
    {
        ntp_caller(&ntp_time);

        #ifdef ENABLE_DST_SWITCH
        if(digitalRead(DST_SWITCH) == HIGH)
        {
            // Switch not activated. do not add an hour
        }
        else
        {
            // Switch activated.. add 1 hour
            if(ntp_time.hour == 23) {
                ntp_time.hour = 0;
            }
            else
            {
                ntp_time.hour++;
            }
        }
        #endif

        update_time = 0; // wait for next timer Event
        if(ntp_time.min % 5 == 0)
        {
            update_disp = 1;
        }

        // determine if display should be on or off
        if((ntp_time.hour >= settings.disp_on_hour) && (ntp_time.hour <= settings.disp_off_hour))
        {
            // within operating hours
            enable_disp = 1;
            if(ntp_time.hour == settings.disp_on_hour)
            {
                // if disp_on_hour.. check min range
                if(ntp_time.min >= settings.disp_on_min)
                {
                    enable_disp = 1;
                }
                else
                {
                    enable_disp = 0;
                }
            }

            if(ntp_time.hour == settings.disp_off_hour)
            {
                //if disp_off_hour.. check min range
                if(ntp_time.min >= settings.disp_off_min)
                {
                    enable_disp = 0;
                }
                else
                {
                    enable_disp = 1;
                }
            }
        }
        else
        {
            enable_disp = 0;
        }// end display check
    }// end time update

    if(update_disp == 1 && enable_disp == 1)
    {
        calculate_bitmap(display, BITMAP_SZ, &ntp_time); // concats bitmaps
        // there is a check in write display to prevent writing same screen
        write_display(display, written_display, BITMAP_SZ); // writes over spi
        update_disp = 0;
    }

    if(trigger_reboot == 1)
    {
        trigger_reboot = 0;
        Serial.println("Reboot Requested");
        delay(2000);
        System.reset();
    }
}
