/* ================================================================
** easydaq.[ch]
**
** Support functions for easyDAQ SERDIO8R relay card.
**
** 31 DEC 2008  Ryan Eustice  Created and written.
** ================================================================
*/

#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <errno.h>
#include <sys/ioctl.h>

#include "perls-common/error.h"
#include "perls-common/serial.h"
#include "perls-common/stdio_util.h"

#include "perls-lcmtypes/senlcm_easydaq_t.h"

#include "easydaq.h"


#define EASYDAQ_READ    (0x41) // ascii 'A'
#define EASYDAQ_SETDIR  (0x42) // ascii 'B'
#define EASYDAQ_WRITE   (0x43) // ascii 'C'
#define EASYDAQ_DELAY   (1e4)  // useconds

int
easydaq_open (const char *device)
{
    int fd = serial_open (device, B9600, PARITY_8N1, 1);
    if (fd < 0) {
        ERROR ("serial_open");
        return -1;
    }
    
    if (serial_set_noncanonical (fd, 0, 1) != 0) {
        ERROR ("serial_set_noncanonical");
        return -1;
    }

    return fd;
}

int
easydaq_close (int fd)
{
    return serial_close (fd);
}

int
easydaq_setdir (int fd, uint8_t byte)
{
    int ret;
    uint8_t cmd[2];

    usleep (EASYDAQ_DELAY);

    cmd[0] = EASYDAQ_SETDIR;
    cmd[1] = byte;
    ret = write (fd, cmd, 2);
    if (ret!=2)
        return -1;
    else
        tcdrain (fd);

    return 0;
}

int
easydaq_read (int fd, uint8_t *byte)
{
    int ret;
    uint8_t cmd[2];

    usleep  (EASYDAQ_DELAY);
    tcflush (fd, TCIFLUSH);

    cmd[0] = EASYDAQ_READ;
    cmd[1] = 0x00;
    ret = write (fd, cmd, 2);
    if (ret != 2)
        return -1;
    else
        tcdrain (fd);

    ret = read (fd, byte, 1);
    if (ret != 1)
        return -1;
    else
        return 0;
}

int
easydaq_write (int fd, uint8_t byte)
{
    int ret;
    uint8_t cmd[2];

    usleep (EASYDAQ_DELAY);

    cmd[0] = EASYDAQ_WRITE;
    cmd[1] = byte;
    ret = write (fd, cmd, 2);
    if (ret != 2)
        return -1;
    else
        tcdrain (fd);

    return 0;
}

void
easydaq_printf (const senlcm_easydaq_t *easydaq)
{
    stdiou_printf_bold ("%-6s  %-25s %-25s %s (*=EA)\n", "Relay#", "Label", "Group", "State");
    for (int i=0; i<8; i++) {
        printf ("%6d  %-25s %-25s %s%c\n", i+1,
                easydaq->relay[i].label, easydaq->relay[i].group, 
                easydaq->relay[i].state ? "ON" : "OFF",
                easydaq->relay[i].exclude_all ? '*' : ' ');
    }
}
