#ifndef __CHECKSUM_H__
#define __CHECKSUM_H__

#include <stdlib.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Calculate the CRC-16-IBM checksum (poly 0x8005 reversed 0xA001
 *
 * If wanting to verify and the crc values follow the data directly
 * increase the size by two and a return value of 0 indicates success.
 */
uint16_t gen_crc16(const uint8_t *data, uint16_t size);


#ifdef __cplusplus
};
#endif

#endif // __CHECKSUM_H__
