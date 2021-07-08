/* NMEA Parser example, that decode data stream from GPS receiver

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nmea_parser.h"
#include "time.h"
#include "math.h"

static const char *TAG = "gps_demo";

// #define TIME_ZONE (+8)   //Beijing Time
#define TIME_ZONE (+9)   //Seoul Time
#define YEAR_BASE (2000) //date in GPS starts from 2000
#define SECS_PER_DAY (60L*60*24)
#define SECS_PER_WEEK (SECS_PER_DAY*7)
#define LEAF_SECOND (18)

uint8_t BAUDRATE_CONFIG[] = "\r\n$PMTK251,115200*1F\r\n";
uint8_t FIX_PERIOD_CONFIG[] = "\r\n$PMTK220,100*2F\r\n";

/**
 * @brief Change to Little endian type
 *
 * @param raw_data the data which will be little endian type
 * @param changed_data the little endian type data will be saved in this pointer
 * @param data_size the size of raw_data(==changed_data), (n Byte)
 */
void make_little_endian(int16_t *raw_data, int16_t *changed data, uint8_t data_size)
{
    
}


// for get "Week-number"
time_t time_frem_YMD(int year, int month, int day) {
    struct tm tm = {0};

    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;

    return mktime(&tm);
}
// for get "Week-number"
int get_gps_week_number(int year, int month, int day) {
  double diff = difftime(time_frem_YMD(year, month, day), time_frem_YMD(1980, 1, 6));
  return (int) (diff / SECS_PER_WEEK);
}

int get_gps_iTOW(int year, int month, int day, int hour, int min, int sec) {
  double diff = difftime(time_frem_YMD(year, month, day), time_frem_YMD(1980, 1, 6));
  return (uint32_t) fmod(diff, SECS_PER_WEEK) + (hour * 3600) + (min * 60) + sec + LEAF_SECOND;
}


/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) // TODO this function will be packet making function
{
    gps_t *gps = NULL;
    nav_pvt_t *nav_pvt = malloc(sizeof(nav_pvt_t));

    switch (event_id) {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;

        //TODO Making NAG-PVT Packet here
        //TODO Need to check the variables with 0 are not used in 3SECONDZ service

        nav_pvt->iTOW = get_gps_iTOW(gps->date.year + YEAR_BASE, gps->date.month, gps->date.day, gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second);
        nav_pvt->date.year = gps->date.year;
        nav_pvt->date.month = gps->date.month;
        nav_pvt->date.day = gps->date.day;
        nav_pvt->time.hour = gps->tim.hour;
        nav_pvt->time.minute = gps->tim.minute;
        nav_pvt->time.second = gps->tim.second;

        nav_pvt->valid = 0;

        nav_pvt->tAcc = 0;
        nav_pvt->nano = 0;

        switch(gps->fix_mode) {
            case GPS_MODE_2D:
                nav_pvt->fixType = NAV_PVT_MODE_2D;
                break;
            case GPS_MODE_3D:
                nav_pvt->fixType = NAV_PVT_MODE_3D;
                break;
            case GPS_MODE_INVALID:
                nav_pvt->fixType = NAV_PVT_MODE_INVALID;
                break;
            default:
                nav_pvt->fixType = 0;
                break;
        }

        nav_pvt->flags = 0;
        nav_pvt->flags2 = 0;

        nav_pvt->numSV = gps->sats_in_use;

        nav_pvt->lat = (int32_t) ((double_t) gps->latitude * (double_t) 10000000); //FIXME When move value to nav_pvt type, the value can be changed, need to fix
        nav_pvt->lon = (int32_t) ((double_t) gps->longitude * (double_t) 10000000); //FIXME When move value to nav_pvt type, the value can be changed, need to fix

        nav_pvt->height = (int32_t) ((double_t) gps->altitude * (double_t) 1000); // nav_pvt - mm, gps - m //FIXME When move value to nav_pvt type, the value can be changed, need to fix

        nav_pvt->hMSL = 0; //FIXME use Geoidal field
        nav_pvt->hAcc = 0;
        nav_pvt->vAcc = 0;
        nav_pvt->velN = 0;
        nav_pvt->velE = 0;
        nav_pvt->velD = 0;

        nav_pvt->gSpeed = (int32_t) ((double_t) gps->speed * (double_t) 1000); //FIXME When move value to nav_pvt type, the value can be changed, need to fix

        nav_pvt->headMot = 0; //FIXME use Course over ground in degrees

        nav_pvt->sAcc = 0;
        nav_pvt->headAcc = 0;
        nav_pvt->pDOP = 0;
        nav_pvt->flags3 = 0;

        for(uint8_t reserved_num = 0; reserved_num < 4; reserved_num++)
        {
            nav_pvt->reserved1[reserved_num] = 0;
        }

        nav_pvt->headVeh = 0;
        nav_pvt->magDec = 0;
        nav_pvt->magAcc = 0;

        //TODO TEST Little endian


        //TODO $PMTK220,100*1F fix period set 10Hz

        free(nav_pvt);

        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}

void app_main(void)
{
    /* NMEA parser configuration */
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    /* init NMEA parser library */
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
    /* register event handler for NMEA parser library */
    nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);

    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // uart_write_bytes(2, BAUDRATE_CONFIG, sizeof(BAUDRATE_CONFIG));
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // uart_write_bytes(2, FIX_PERIOD_CONFIG, sizeof(FIX_PERIOD_CONFIG));



    // vTaskDelay(10000 / portTICK_PERIOD_MS);

    // /* unregister event handler */
    // nmea_parser_remove_handler(nmea_hdl, gps_event_handler);
    // /* deinit NMEA parser library */
    // nmea_parser_deinit(nmea_hdl);
}
