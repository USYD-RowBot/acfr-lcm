#ifndef __EASYDAQ_H__
#define __EASYDAQ_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int
easydaq_open (const char *device);

int
easydaq_close (int fd);

int
easydaq_setdir (int fd, uint8_t byte);

int
easydaq_read (int fd, uint8_t *byte);

int
easydaq_write (int fd, uint8_t byte);

void
easydaq_printf (const senlcm_easydaq_t *easydaq);

#ifdef __cplusplus
}
#endif

#endif // __EASYDAQ_H__
