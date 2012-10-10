// file: listener.c
//
// LCM example program.
//
// compile with:
//  $ gcc -o listener listener.c -llcm
//
// If using GNU/Linux, you can also use pkg-config:
//  $ gcc -o listener listener.c `pkg-config --cflags --libs lcm`

#include <stdio.h>
#include <inttypes.h>

#include "perls-lcmtypes/bot_core_raw_t.h"

static void
my_handler (const lcm_recv_buf_t *rbuf, const char * channel, 
            const bot_core_raw_t * msg, void * user)
{
    static int count = 0;
    printf ("Received %d raw_t on channel \"%s\":\n", ++count, channel);
    printf ("raw->length = %d\n", msg->length);
}

int
main (int argc, char ** argv)
{
    lcm_t * lcm;

    lcm = lcm_create (NULL);
    if (!lcm)
        return 1;

    bot_core_raw_t_subscribe (lcm, "EXAMPLE", &my_handler, NULL);

    while (1)
        lcm_handle (lcm);

    lcm_destroy (lcm);
    return 0;
}

