

#include <stdio.h>
#include <readline/readline.h>
#include <readline/history.h>
#include "perls-lcmtypes/acfrlcm_auv_camera_trigger_t.h"

// command messages
#define SET_FREQ				1
#define SET_DELAY				2
#define	SET_WIDTH				3
#define SET_STATE				4
#define SET_ALL					5

int main()
{

    int exit = 0;
    char *line;
    acfrlcm_auv_camera_trigger_t ct;
    ct.freq = 1;
    ct.strobeDelayUs = 0;
    ct.pulseWidthUs = 10;
    ct.enabled = 0;
    lcm_t *lcm = lcm_create(NULL);

    while(!exit)
    {
        line = readline(">");
        switch(line[0])
        {
        case 'q':
        case 'Q':
            ct.enabled = 0;
            ct.command = SET_STATE;
            exit = 1;
            break;
        case 'f':
        case 'F':
            sscanf(&line[1], "%lf", &ct.freq);
            ct.command = SET_ALL;
            break;
        case 'd':
        case 'D':
            sscanf(&line[1], "%d", &ct.strobeDelayUs);
            ct.command = SET_ALL;
            break;
        case 'w':
        case 'W':
            sscanf(&line[1], "%d", &ct.pulseWidthUs);
            ct.command = SET_ALL;
            break;
        case 'e':
        case 'E':
            sscanf(&line[1], "%d", &ct.enabled);
            ct.command = SET_STATE;
            break;
        }

        printf("f=%f, d=%d, w=%d, e=%d\n", ct.freq, ct.strobeDelayUs, ct.pulseWidthUs, ct.enabled);
        acfrlcm_auv_camera_trigger_t_publish(lcm, "NGA.CAMERA_TRIGGER", &ct);
    }

    return 0;
}





