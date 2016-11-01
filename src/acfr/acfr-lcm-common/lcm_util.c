#include "lcm_util.h"


int
lcmu_handle_timeout (lcm_t *lcm, struct timeval *timeout)
{
    int fd = lcm_get_fileno (lcm);

    fd_set rfds;
    FD_ZERO (&rfds);
    FD_SET (fd, &rfds);

    int ret = select (fd + 1, &rfds, NULL, NULL, timeout);

    if (ret == -1)
    {
#ifdef LINUX
        if (errno == EINTR)
            return lcmu_handle_timeout (lcm, timeout);
#endif
        if (errno != EINTR)
            fprintf(stderr, "select()");
    }
    else if (ret == 0)
    {
        /* Timeout */
    }
    else
    {
        /* We have data. */
        if (FD_ISSET (fd, &rfds))
            lcm_handle (lcm);
    }

    return ret;
}


