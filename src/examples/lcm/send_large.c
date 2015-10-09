// file: send_large.c
//
// LCM example program.
//
// compile with:
//  $ gcc -o send_large send_message.c -llcm
//
// If using GNU/Linux, you can also use pkg-config:
//  $ gcc -o send_large send_message.c `pkg-config --cflags --libs lcm`

#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include "perls-lcmtypes/bot_core_raw_t.h"

//#define LENGTH (190373) /* 1 MB */
#define LENGTH (1000000) /* 1 MB */

static void
send_message (lcm_t * lcm, long length)
{
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);

    uint8_t *data = malloc (length);
    bot_core_raw_t raw =
    {
        .utime = tv.tv_sec*1E6+tv.tv_usec,
        .length = length,
        .data = data,
    };
    bot_core_raw_t_publish (lcm, "EXAMPLE", &raw);
    free (data);
}

int
main (int argc, char ** argv)
{
    lcm_t * lcm;

    lcm = lcm_create (NULL);
    if (!lcm)
        return 1;

    if (argc == 1)
        send_message (lcm, LENGTH);
    else
        send_message (lcm, atoi (argv[1]));

    lcm_destroy (lcm);
    return 0;
}
