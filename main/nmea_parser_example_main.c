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

#define TIME_ZONE (+9)   //Seoul Time
#define YEAR_BASE (2000) //date in GPS starts from 2000
#define SECS_PER_DAY (60L*60*24)
#define SECS_PER_WEEK (SECS_PER_DAY*7)
#define LEAF_SECOND (18)
#define MAX_SECS_OF_WEEK (604799) // 0 ~ 604799, 604800 = 0

#define _UbxHeader_1    0xB5
#define _UbxHeader_2    0x62

uint8_t BAUDRATE_CONFIG[] = "\r\n$PMTK251,115200*1F\r\n";
uint8_t FIX_PERIOD_CONFIG[] = "\r\n$PMTK220,100*2F\r\n";

/**
 * @brief Change to Little endian type
 *
 * @param raw_data the data which will be little endian type
 * @param changed_data the little endian type data will be saved in this pointer
 * @param array_size the size of raw_data(==changed_data), (n Byte)
 * @param array_address_buffer this agur will be increased by array_size and returned
 */
void make_little_endian(int32_t *raw_data, uint8_t *changed_data, uint8_t array_size, uint32_t *array_address_buffer)
{
    for(int count = (array_size - 1); count >= 0; count--)
	{
		*(changed_data + count) = ((*raw_data) >> (8 * count)) & 0xFF;
	}
    *array_address_buffer += array_size;
}

/**
 * @brief Make checksum for UBX protocol
 *
 * @param pucBuf the data which will be little endian type
  */
uint16_t usUbxCalcCheckSum(uint8_t *pucBuf)
{
    uint8_t ucChk1 = 0;
    uint8_t ucChk2 = 0;
    uint16_t usCnt = 0;
    uint16_t usLength = 0;

    if((pucBuf[0] != _UbxHeader_1) || (pucBuf[1] != _UbxHeader_2))
    {
        printf("Chksum: Header mismatch\n\r");
        return 0;
    }
    usLength = pucBuf[5];
    usLength <<= 8;
    usLength |= pucBuf[4];
    usLength += 4;
    if(usLength==0 || usLength > 1000)
        return 0;
    for(usCnt = 0; usCnt < usLength; usCnt++)
    {
        ucChk1 = ucChk1 + pucBuf[usCnt + 2];
        ucChk2 = ucChk1 + ucChk2;
    }
    usLength = ucChk1;
    usLength <<= 8;
    usLength |= ucChk2;
    return usLength;
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

int get_gps_iTOW(int year, int month, int day, int hour, int min, int sec, int thousand) {
	double diff = difftime(time_frem_YMD(year, month, day), time_frem_YMD(1980, 1, 6));
	uint32_t buffer =  (int) (fmod(diff, SECS_PER_WEEK) + (hour * 3600) + (min * 60) + sec + LEAF_SECOND);
	if(buffer > MAX_SECS_OF_WEEK)
	{
		buffer -= (MAX_SECS_OF_WEEK + 1);
	}
	return (uint32_t) (buffer * 1000) + thousand;
}


/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    gps_t *gps = NULL;
    nav_pvt_t *nav_pvt_struct = malloc(sizeof(nav_pvt_t));

    uint8_t nav_pvt_array[sizeof(nav_pvt_t)] = {0, };
    uint32_t data_length_buffer = 0;

    switch (event_id) {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;

#if (__GNSS_COORDINATE_MODE == 2)
        printf("%s", (char *)event_data);

#elif (__GNSS_COORDINATE_MODE == 1)
        /* print information parsed from GPS statements */
        ESP_LOGI(TAG, "%d/%d/%d %d:%d:%d => "
                 "\t\tlatitude   = %.05f°N"
                 "\t\tlongitude = %.05f°E"
                 "\t\taltitude   = %.02fm"
                 "\t\tspeed      = %fm/s\r\n",
                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                 gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                 gps->latitude, gps->longitude, gps->altitude, gps->speed);

#elif (__GNSS_COORDINATE_MODE == 0)
        //TODO Need to check the variables with 0 are not used in 3SECONDZ service

        nav_pvt_struct->header = 0xB562;
        nav_pvt_struct->class = 0x01;
        nav_pvt_struct->id = 0x07;

        // packet header initialize
        nav_pvt_array[0] = (nav_pvt_struct->header >> 8) & 0xFF;
        nav_pvt_array[1] = nav_pvt_struct->header & 0xFF;
        nav_pvt_array[2] = nav_pvt_struct->class;
        nav_pvt_array[3] = nav_pvt_struct->id;
        nav_pvt_array[4] = 0x5C; // nav_pvt payload length
        nav_pvt_array[5] = 0x00;
        data_length_buffer = sizeof(nav_pvt_struct->header) + sizeof(nav_pvt_struct->class) + sizeof(nav_pvt_struct->id) + 2;

        nav_pvt_struct->iTOW = get_gps_iTOW(gps->date.year + YEAR_BASE, gps->date.month, gps->date.day, gps->tim.hour, gps->tim.minute, gps->tim.second, gps->tim.thousand);
        nav_pvt_struct->date.year = gps->date.year + YEAR_BASE;
        nav_pvt_struct->date.month = gps->date.month;
        nav_pvt_struct->date.day = gps->date.day;
        nav_pvt_struct->time.hour = gps->tim.hour;
        nav_pvt_struct->time.minute = gps->tim.minute;
        nav_pvt_struct->time.second = gps->tim.second;
        nav_pvt_struct->time.thousand = gps->tim.thousand;

        nav_pvt_struct->valid = 0;

        nav_pvt_struct->tAcc = 0;
        nav_pvt_struct->nano = 0;

        switch(gps->fix_mode) {
            case GPS_MODE_2D:
                nav_pvt_struct->fixType = NAV_PVT_MODE_2D;
                break;
            case GPS_MODE_3D:
                nav_pvt_struct->fixType = NAV_PVT_MODE_3D;
                break;
            case GPS_MODE_INVALID:
                nav_pvt_struct->fixType = NAV_PVT_MODE_INVALID;
                break;
            default:
                nav_pvt_struct->fixType = 0;
                break;
        }

        nav_pvt_struct->flags = 0;
        nav_pvt_struct->flags2 = 0;

        nav_pvt_struct->numSV = gps->sats_in_use;

        nav_pvt_struct->lon = gps->longitude;
        nav_pvt_struct->lat = gps->latitude;
        nav_pvt_struct->height = (int32_t) ((double_t) gps->altitude * (double_t) 1000); // nav_pvt - mm, gps - m //FIXME When move value to nav_pvt type, the value can be changed, need to fix

        nav_pvt_struct->hMSL = 0; //FIXME use Geoidal field
        nav_pvt_struct->hAcc = 0;
        nav_pvt_struct->vAcc = 0;
        nav_pvt_struct->velN = 0;
        nav_pvt_struct->velE = 0;
        nav_pvt_struct->velD = 0;

        nav_pvt_struct->gSpeed = (int32_t) ((double_t) gps->speed * (double_t) 1000); //FIXME When move value to nav_pvt type, the value can be changed, need to fix

        nav_pvt_struct->headMot = 0; //FIXME use Course over ground in degrees

        nav_pvt_struct->sAcc = 0;
        nav_pvt_struct->headAcc = 0;
        nav_pvt_struct->pDOP = 0;
        nav_pvt_struct->flags3 = 0;

        for(uint8_t reserved_num = 0; reserved_num < 4; reserved_num++)
        {
            nav_pvt_struct->reserved1[reserved_num] = 0;
        }

        nav_pvt_struct->headVeh = 0;
        nav_pvt_struct->magDec = 0;
        nav_pvt_struct->magAcc = 0;



        make_little_endian((int32_t) &(nav_pvt_struct->iTOW), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->iTOW), &data_length_buffer);

        make_little_endian((int32_t) &(nav_pvt_struct->date.year), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->date.year), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->date.month), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->date.month), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->date.day), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->date.day), &data_length_buffer);

        make_little_endian((int32_t) &(nav_pvt_struct->time.hour), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->time.hour), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->time.minute), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->time.minute), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->time.second), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->time.second), &data_length_buffer);

        make_little_endian((int32_t) &(nav_pvt_struct->valid), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->valid), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->tAcc), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->tAcc), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->nano), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->nano), &data_length_buffer);

        make_little_endian((int32_t) &(nav_pvt_struct->fixType), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->fixType), &data_length_buffer);

        make_little_endian((int32_t) &(nav_pvt_struct->flags), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->flags), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->flags2), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->flags2), &data_length_buffer);

        make_little_endian((int32_t) &(nav_pvt_struct->numSV), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->numSV), &data_length_buffer);

        make_little_endian((int32_t) &(nav_pvt_struct->lon), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->lon), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->lat), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->lat), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->height), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->height), &data_length_buffer);

        make_little_endian((int32_t) &(nav_pvt_struct->hMSL), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->hMSL), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->hAcc), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->hAcc), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->vAcc), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->vAcc), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->velN), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->velN), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->velE), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->velE), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->velD), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->velD), &data_length_buffer);

        make_little_endian((int32_t) &(nav_pvt_struct->gSpeed), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->gSpeed), &data_length_buffer);

        make_little_endian((int32_t) &(nav_pvt_struct->headMot), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->headMot), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->sAcc), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->sAcc), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->headAcc), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->headAcc), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->pDOP), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->pDOP), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->flags3), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->flags3), &data_length_buffer);

        make_little_endian((int32_t) &(nav_pvt_struct->reserved1[0]), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->reserved1[0]), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->reserved1[1]), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->reserved1[1]), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->reserved1[2]), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->reserved1[2]), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->reserved1[3]), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->reserved1[3]), &data_length_buffer);

        make_little_endian((int32_t) &(nav_pvt_struct->headVeh), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->headVeh), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->magDec), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->magDec), &data_length_buffer);
        make_little_endian((int32_t) &(nav_pvt_struct->magAcc), &(nav_pvt_array[data_length_buffer]), sizeof(nav_pvt_struct->magAcc), &data_length_buffer);

        uint16_t checksum_buffer = 0;
        checksum_buffer = usUbxCalcCheckSum(nav_pvt_array);

        nav_pvt_array[data_length_buffer] = (checksum_buffer >> 8) & 0xFF;
        data_length_buffer += 1;
        nav_pvt_array[data_length_buffer] = checksum_buffer & 0xFF;
        data_length_buffer += 1;


        for(int i = 0; i < data_length_buffer; i++) // TODO PACKET MAKING TESTED
        {
            printf("%02x ", nav_pvt_array[i]);
        }
        printf("\r\n");
        // printf(" // data_length_buffer - %d // nav_pvt_t - %d\r\n", data_length_buffer, sizeof(nav_pvt_t));

#endif

        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;

    free(nav_pvt_struct);
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
