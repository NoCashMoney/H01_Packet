// Copyright 2015-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_types.h"
#include "esp_event.h"
#include "esp_err.h"
#include "driver/uart.h"

#define __GNSS_COORDINATE_MODE (2) // 0: NAV_PVT OUTPUT  1: COORDINATE OUTPUT  2: GNSS DIRECT MODE

#define GPS_MAX_SATELLITES_IN_USE (12)
#define GPS_MAX_SATELLITES_IN_VIEW (16)

/**
 * @brief Declare of NMEA Parser Event base
 *
 */
ESP_EVENT_DECLARE_BASE(ESP_NMEA_EVENT);

/**
 * @brief GPS fix type
 *
 */
typedef enum {
    GPS_FIX_INVALID, /*!< Not fixed */
    GPS_FIX_GPS,     /*!< GPS */
    GPS_FIX_DGPS,    /*!< Differential GPS */
} gps_fix_t;

/**
 * @brief GPS fix mode
 *
 */
typedef enum {
    GPS_MODE_INVALID = 1, /*!< Not fixed */
    GPS_MODE_2D,          /*!< 2D GPS */
    GPS_MODE_3D           /*!< 3D GPS */
} gps_fix_mode_t;

/**
 * @brief GPS satellite information
 *
 */
typedef struct {
    uint8_t num;       /*!< Satellite number */
    uint8_t elevation; /*!< Satellite elevation */
    uint16_t azimuth;  /*!< Satellite azimuth */
    uint8_t snr;       /*!< Satellite signal noise ratio */
} gps_satellite_t;

/**
 * @brief GPS time
 *
 */
typedef struct {
    uint8_t hour;      /*!< Hour */
    uint8_t minute;    /*!< Minute */
    uint8_t second;    /*!< Second */
    uint16_t thousand; /*!< Thousand */
} gps_time_t;

/**
 * @brief GPS date
 *
 */
typedef struct {
    uint8_t day;   /*!< Day (start from 1) */
    uint8_t month; /*!< Month (start from 1) */
    uint16_t year; /*!< Year (start from 2000) */
} gps_date_t;

/**
 * @brief NMEA Statement
 *
 */
typedef enum {
    STATEMENT_UNKNOWN = 0, /*!< Unknown statement */
    STATEMENT_GGA,         /*!< GGA */
    STATEMENT_GSA,         /*!< GSA */
    STATEMENT_RMC,         /*!< RMC */
    STATEMENT_GSV,         /*!< GSV */
    STATEMENT_GLL,         /*!< GLL */
    STATEMENT_VTG          /*!< VTG */
} nmea_statement_t;

/**
 * @brief GPS object
 *
 */
typedef struct {
    int32_t latitude;                                                /*!< Latitude (degrees) */
    int32_t longitude;                                               /*!< Longitude (degrees) */
    float altitude;                                                /*!< Altitude (meters) */
    gps_fix_t fix;                                                 /*!< Fix status */
    uint8_t sats_in_use;                                           /*!< Number of satellites in use */
    gps_time_t tim;                                                /*!< time in UTC */
    gps_fix_mode_t fix_mode;                                       /*!< Fix mode */
    uint8_t sats_id_in_use[GPS_MAX_SATELLITES_IN_USE];             /*!< ID list of satellite in use */
    float dop_h;                                                   /*!< Horizontal dilution of precision */
    float dop_p;                                                   /*!< Position dilution of precision  */
    float dop_v;                                                   /*!< Vertical dilution of precision  */
    uint8_t sats_in_view;                                          /*!< Number of satellites in view */
    gps_satellite_t sats_desc_in_view[GPS_MAX_SATELLITES_IN_VIEW]; /*!< Information of satellites in view */
    gps_date_t date;                                               /*!< Fix date */
    bool valid;                                                    /*!< GPS validity */
    float speed;                                                   /*!< Ground speed, unit: m/s */
    float cog;                                                     /*!< Course over ground */
    float variation;                                               /*!< Magnetic variation */
} gps_t;

/**
 * @brief Configuration of NMEA Parser
 *
 */
typedef struct {
    struct {
        uart_port_t uart_port;        /*!< UART port number */
        uint32_t rx_pin;              /*!< UART Rx Pin number */
        uint32_t baud_rate;           /*!< UART baud rate */
        uart_word_length_t data_bits; /*!< UART data bits length */
        uart_parity_t parity;         /*!< UART parity */
        uart_stop_bits_t stop_bits;   /*!< UART stop bits length */
        uint32_t event_queue_size;    /*!< UART event queue size */
    } uart;                           /*!< UART specific configuration */
} nmea_parser_config_t;

/**
 * @brief NMEA Parser Handle
 *
 */
typedef void *nmea_parser_handle_t;

//TODO NAV-PVT type
typedef enum {
    NAV_PVT_MODE_INVALID,       /*!< Not fixed */
    NAV_PVT_MODE_DEAD,          /*!< Dead-reckoning */
    NAV_PVT_MODE_2D,            /*!< 2D GPS */
    NAV_PVT_MODE_3D,            /*!< 3D GPS */
    NAV_PVT_MODE_COMBINED,      /*!< GNSS + dead-reckoning */
    NAV_PVT_MODE_TIMEONLY       /*!< Time only fixed */
} nav_pvt_fix_mode_t;

typedef struct {
    uint16_t header;		        // Packet header
    uint8_t class;			        // Class
    uint8_t id;			        	// Id
    uint16_t length;                // payload length
    uint32_t iTOW;                  // Time of week (sec)
    gps_date_t date;                // Date from GPS (UTC+0)
    gps_time_t time;                // Time from GPS (UTC+0)
    uint8_t valid;                  // Validity flags
    uint32_t tAcc;                  // Time accuracy estimate (UTC+0) !not used!
    uint32_t nano;                  // Fraction of second (UTC+0) !not used!
    uint8_t fixType;                // 0: nofix  1: Dead-reckoning  2: 2D-fix  3: 3D-fix  4: GNSS + Dead-reckoning  5: time only fix
    uint8_t flags;                  // Fix status flags
    uint8_t flags2;                 // Additional flags
    uint8_t numSV;                  // Number of satellites used in Nav Solution
    int32_t lon;                    // Longitude
    int32_t lat;                    // Latitude
    int32_t height;                 // Height above ellipsoid
    int32_t hMSL;                   // Height above mean sea level
    uint32_t hAcc;                  // Horizontal accuracy estimate
    uint32_t vAcc;                  // Vertical accuracy estimate
    int32_t velN;                   // NED north velocity
    int32_t velE;                   // NED east velocity
    int32_t velD;                   // NED down velocity
    int32_t gSpeed;                 // Ground Speed (2-D)
    int32_t headMot;                // Heading of motion (2-D)
    uint32_t sAcc;                  // Speed accuracy estimate
    uint32_t headAcc;               // Heading accuracy estimate
    uint16_t pDOP;                  // Position DOP
    uint16_t flags3;                // Additional flags
    uint8_t reserved1[4];           // Reserved
    int32_t headVeh;                // Heading of vehicle (2-D)
    int16_t magDec;                 // Magnetic declination
    uint16_t magAcc;                // Magnetic declination accuracy
    uint16_t checksum;             // Checksum A CK_A = CK_A + Buffer[I]
} nav_pvt_t;



/**
 * @brief Default configuration for NMEA Parser
 *
 */
#define NMEA_PARSER_CONFIG_DEFAULT()       \
    {                                      \
        .uart = {                          \
            .uart_port = UART_NUM_2,       \
            .rx_pin = 27,                   \
            .baud_rate = 115200,             \
            .data_bits = UART_DATA_8_BITS, \
            .parity = UART_PARITY_DISABLE, \
            .stop_bits = UART_STOP_BITS_1, \
            .event_queue_size = 256         \
        }                                  \
    }

/**
 * @brief NMEA Parser Event ID
 *
 */
typedef enum {
    GPS_UPDATE, /*!< GPS information has been updated */
    GPS_UNKNOWN /*!< Unknown statements detected */
} nmea_event_id_t;

/**
 * @brief Init NMEA Parser
 *
 * @param config Configuration of NMEA Parser
 * @return nmea_parser_handle_t handle of NMEA parser
 */
nmea_parser_handle_t nmea_parser_init(const nmea_parser_config_t *config);

/**
 * @brief Deinit NMEA Parser
 *
 * @param nmea_hdl handle of NMEA parser
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t nmea_parser_deinit(nmea_parser_handle_t nmea_hdl);

/**
 * @brief Add user defined handler for NMEA parser
 *
 * @param nmea_hdl handle of NMEA parser
 * @param event_handler user defined event handler
 * @param handler_args handler specific arguments
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_NO_MEM: Cannot allocate memory for the handler
 *  - ESP_ERR_INVALIG_ARG: Invalid combination of event base and event id
 *  - Others: Fail
 */
esp_err_t nmea_parser_add_handler(nmea_parser_handle_t nmea_hdl, esp_event_handler_t event_handler, void *handler_args);

/**
 * @brief Remove user defined handler for NMEA parser
 *
 * @param nmea_hdl handle of NMEA parser
 * @param event_handler user defined event handler
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALIG_ARG: Invalid combination of event base and event id
 *  - Others: Fail
 */
esp_err_t nmea_parser_remove_handler(nmea_parser_handle_t nmea_hdl, esp_event_handler_t event_handler);

#ifdef __cplusplus
}
#endif
