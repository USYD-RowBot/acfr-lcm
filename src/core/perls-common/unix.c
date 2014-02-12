#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "unix.h"

/* Function with behaviour like `mkdir -p'  */
int
unix_mkpath (const char *s, mode_t mode)
{
    char *q, *r = NULL, *path = NULL, *up = NULL;
    int rv;
 
    rv = -1;
    if (strcmp (s, ".") == 0 || strcmp (s, "/") == 0)
        return (0);
 
    if ((path = strdup (s)) == NULL)
        exit (EXIT_SUCCESS);
 
    if ((q = strdup(s)) == NULL)
        exit (EXIT_SUCCESS);
 
    if ((r = dirname (q)) == NULL)
        goto out;
 
    if ((up = strdup (r)) == NULL)
        exit (EXIT_SUCCESS);
 
    if ((unix_mkpath (up, mode) == -1) && (errno != EEXIST))
        goto out;
 
    if ((mkdir (path, mode) == -1) && (errno != EEXIST))
        rv = -1;
    else
        rv = 0;
 
  out:
    if (up != NULL)
        free (up);
    free (q);
    free (path);
    return (rv);
}

int
unix_pidstat (pid_t pid)
{
    char procdir[64];
    snprintf (procdir, sizeof (procdir), "/proc/%d", pid);
    struct stat info;
    return stat (procdir, &info);
}
