/*
 * gps_buff.h
 *
 *  Created on: Apr 16, 2019
 *      Author: Luke
 */

#ifndef APPLICATION_GPS_GPS_BUFF_H_
#define APPLICATION_GPS_GPS_BUFF_H_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "stddef.h"
#include "stdint.h"
#include "string.h"

/**
 * \addtogroup      GPS_NMEA
 * \{
 */

/**
 * \defgroup        GPS_NMEA_BUFFER Ring buffer
 * \brief           Optional ring buffer for received GPS data
 * \{
 */

/**
 * \brief           GPS buffer structure
 */
typedef struct gps_buff {
    size_t size;                                /*!< Size of buffer in units of bytes */
    size_t in;                                  /*!< Input pointer to save next value */
    size_t out;                                 /*!< Output pointer to read next value */
    uint8_t* buff;                              /*!< Pointer to buffer data array */
    uint8_t flags;                              /*!< Flags for buffer */
} gps_buff_t;

uint8_t     gps_buff_init(gps_buff_t* buff, void* data, size_t len);
void        gps_buff_free(gps_buff_t* buff);
size_t      gps_buff_write(gps_buff_t* buff, const void* data, size_t count);
size_t      gps_buff_read(gps_buff_t* buff, void* data, size_t count);
size_t      gps_buff_get_free(gps_buff_t* buff);
size_t      gps_buff_get_full(gps_buff_t* buff);
void        gps_buff_reset(gps_buff_t* buff);
size_t      gps_buff_peek(gps_buff_t* buff, size_t skip_count, void* data, size_t count);
void *      gps_buff_get_linear_block_address(gps_buff_t* buff);
size_t      gps_buff_get_linear_block_length(gps_buff_t* buff);
size_t      gps_buff_skip(gps_buff_t* buff, size_t len);

/**
 * \}
 */

/**
 * \}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* APPLICATION_GPS_GPS_BUFF_H_ */
