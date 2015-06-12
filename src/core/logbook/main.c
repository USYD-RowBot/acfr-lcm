#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <glib.h>

#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"

#include "perls-lcmtypes/perllcm_logbook_t.h"

#define DEFAULT_EDITOR "nano"

static int done = 0;

typedef struct state state_t;
struct state
{
    // common
    getopt_t *gopt;
    lcm_t *lcm;

    // server
    FILE *fd;
    bool  longfmt;
    unsigned int logentry;
    unsigned int lognum;
    bool verbose;

    // client
    char editor[256];
};


static void
my_signal_handler (int signum)
{
    done = 1;
}

static void
perllcm_logbook_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                            const perllcm_logbook_t *msg, void *user)
{
    state_t *state = user;

    char timestr[128];
    if ( 0 == timeutil_strftime (timestr, sizeof timestr, "%F %T", msg->utime))
    {
        ERROR ("timeutil_strftime()");
        return;
    }


    if (state->longfmt)
    {
        char datestr[128];
        if ( 0 == timeutil_strftime (datestr, sizeof datestr, "%a, %d %b %Y", msg->utime))
        {
            ERROR ("timeutil_strftime()");
            return;
        }

        char header[512];
        if (sizeof header <= snprintf (header, sizeof header,
                                       "e%u | %s@%s | %s (%s) | %"PRId64,
                                       state->logentry, msg->username, msg->hostname, timestr, datestr, msg->utime))
        {
            ERROR ("header was truncated");
            return;
        }
        else
            state->logentry++;

        const char sep[] = "-------------------------------------------------------------------------";
        const char fmt[] = "%s\n%s\n\n%s\n\n";
        fprintf (state->fd, fmt, sep, header, msg->message);
        if (state->verbose)
            printf (fmt, sep, header, msg->message);
    }
    else
    {
        // strip message to a single line for compact mode
        char *message = strdup (msg->message);
        g_strstrip (message);
        char *ptrA=message, *ptrB=message;
        bool whitespace=false;
        do
        {
            if (g_ascii_isalnum (*ptrB) || g_ascii_ispunct (*ptrB))
            {
                *ptrA++ = *ptrB;
                whitespace = false;
            }
            else
            {
                if (whitespace)
                {
                    // skip repeated whitespace
                }
                else   // first occurrence of ' ','\r','\n',etc
                {
                    *ptrA++ = ' ';
                    whitespace = true;
                }
            }

        }
        while (*ptrB++ != '\0');
        *ptrA = '\0';

        const char fmt[] = "%s  %s\n";
        fprintf (state->fd, fmt, timestr, message);
        if (state->verbose)
            printf (fmt, timestr, message);

        g_free (message);
    }
    fflush (state->fd);
}

static int
open_logfile (state_t *state)
{
    // close existing log file
    if (state->fd && 0!=fclose (state->fd))
    {
        PERROR ("fclose(state->fd)");
        return -1;
    }

    // determine log file name
    char filename[PATH_MAX];
    if (state->gopt->extraargs->len)
    {
        timeutil_strftime (filename, sizeof filename, state->gopt->extraargs->pdata[0],
                           timestamp_now ());
    }
    else
    {
        timeutil_strftime (filename, sizeof filename, "LOGBOOK-%Y-%m-%d_%H:%M:%S",
                           timestamp_now ());
    }
    int nseconds = getopt_get_int (state->gopt, "new");
    char *tmp = strdup (filename);
    if (nseconds)
    {
        snprintf (filename, sizeof filename - strlen (filename), "%s-L%04u",
                  tmp, state->lognum);
    }
    else
    {
        snprintf (filename, sizeof filename - strlen (filename), "%s",
                  tmp);
    }
    g_free (tmp);

    // open log file
    char *dirname = g_path_get_dirname (filename);
    if (g_mkdir_with_parents (dirname, 0775) < 0)
    {
        PERROR ("g_mkdir_with_parents (%s, 0775)", dirname);
        return -1;
    }

    if (g_file_test (filename, G_FILE_TEST_EXISTS) &&
            !(getopt_get_bool (state->gopt, "force") || getopt_get_bool (state->gopt, "append")))
    {
        fprintf (stderr, "%s already exists, use --force to overwrite or --append to append.\n", filename);
        return -1;
    }

    char message[1024] = {'\0'};
    if (getopt_get_bool (state->gopt, "append"))
    {
        state->fd = fopen (filename, "a");
        if (!state->fd)
        {
            PERROR ("fopen(%s, \"a\")", filename);
            return -1;
        }

        char *contents;
        GError *error;
        if (!g_file_get_contents (filename, &contents, NULL, &error))
        {
            ERROR ("unable to read file \"%s\" [%s]", filename, error->message);
            g_error_free (error);
            return -1;
        }

        // check for file format compatability LONG vs. SHORT mode
        const char needle[] = "----\ne";
        char *ptr = g_strrstr (contents, needle);
        if (contents && ptr==NULL && state->longfmt)
        {
            ERROR ("Incompatibility detected, appending SHORT format file in LONG mode");
            return -1;
        }
        if (contents && ptr!=NULL && state->longfmt==false)
        {
            ERROR ("Incompatibility detected, appending LONG format file in SHORT mode");
            return -1;
        }

        // parse logentry for LONG format log files
        if (ptr!=NULL)
        {
            char *fmt = g_strconcat (needle, "%d", NULL);
            if (1!=sscanf (ptr, fmt, &state->logentry))
            {
                ERROR ("unable to parse log entry number, ptr=[%s]", ptr);
                g_free (fmt);
                return -1;
            }
            else
                state->logentry++;
            g_free (fmt);
        }

        g_free (contents);
        snprintf (message, sizeof message,
                  "Appending log file \"%s\" %s format", filename, state->longfmt ? "LONG" : "SHORT");
    }
    else
    {
        state->fd = fopen (filename, "w");
        if (!state->fd)
        {
            PERROR ("fopen(%s, \"w\")", filename);
            return -1;
        }
        snprintf (message, sizeof message,
                  "Opening log file \"%s\" %s format", filename, state->longfmt ? "LONG" : "SHORT");
    }
    state->lognum++;
    printf ("%s\n", message);

    const perllcm_logbook_t entry =
    {
        .utime = timestamp_now (),
        .username = "logbook",
        .hostname = "localhost",
        .message = message,
    };
    const char *channel = getopt_get_string (state->gopt, "channel");
    perllcm_logbook_t_publish (state->lcm, channel, &entry);


    return 0;
}

static int
server (state_t *state)
{
    if (getopt_get_bool (state->gopt, "daemon"))
        daemon_fork ();

    state->verbose = getopt_get_bool (state->gopt, "verbose");

    // LCM channels
    const char *channel = getopt_get_string (state->gopt, "channel");
    perllcm_logbook_t_subscribe (state->lcm, channel, &perllcm_logbook_t_callback, state);

    // open log file
    if (getopt_get_bool (state->gopt, "longfmt"))
        state->longfmt = true;
    if (0!=open_logfile (state))
    {
        ERROR ("open_logfile ()");
        exit (EXIT_FAILURE);
    }

    int64_t seconds_open = timestamp_now () / 1000000;
    int64_t nseconds = getopt_get_int (state->gopt, "new");
    while (!done)
    {
        // check if we need to start a new log file
        int64_t seconds_now = timestamp_now () / 1000000;
        if (nseconds && (seconds_now > seconds_open) && (seconds_now % nseconds)==0)
        {
            if (0!=open_logfile (state))
            {
                ERROR ("open_logfile ()");
                exit (EXIT_FAILURE);
            }
            seconds_open = seconds_now;
        }

        struct timeval timeout =
        {
            .tv_sec = 0,
            .tv_usec = 250000,
        };
        lcmu_handle_timeout (state->lcm, &timeout);
    }

    printf ("\nGoodbye\n");

    return 0;
}

static int
client (state_t *state)
{
    int64_t utime = timestamp_now ();

    char username[128];
    if (getopt_has_flag (state->gopt, "username"))
        snprintf (username, sizeof username, "%s", getopt_get_string (state->gopt, "username"));
    else
        snprintf (username, sizeof username, "%s", g_get_user_name ());

    char hostname[HOST_NAME_MAX];
    snprintf (hostname, sizeof hostname, "%s", g_get_host_name ());

    char *message = NULL;
    if (getopt_has_flag (state->gopt, "message"))
        message = strdup (getopt_get_string (state->gopt, "message"));
    else if (getopt_has_flag (state->gopt, "file"))   // grab from file
    {
        gsize len;
        char *contents;
        GError *error;
        const char *file = getopt_get_string (state->gopt, "file");
        if (g_file_get_contents (file, &contents, &len, &error))
            message = strdup (contents);
        else
        {
            ERROR ("unable to read file \"%s\", message not logged", file);
            return -1;
        }
        g_free (contents);
    }
    else   // use external editor
    {
        char tmpfile[256];
        snprintf (tmpfile, sizeof tmpfile, "%s/logbook-tmp%"PRId64".txt", g_get_tmp_dir (), utime);

        char cmd[256];
        snprintf (cmd, sizeof cmd, "%s %s", state->editor, tmpfile);
        if (system (cmd))
        {
            ERROR ("external editor command failed, message not logged");
            return -1;
        }

        gsize len;
        char *contents;
        GError *error;
        if (g_file_get_contents (tmpfile, &contents, &len, &error))
            message = strdup (contents);
        else
        {
            ERROR ("unable to read external editor data, message not logged");
            return -1;
        }
        g_free (contents);
        remove (tmpfile);
    }

    const perllcm_logbook_t entry =
    {
        .utime = timestamp_now (),
        .username = username,
        .hostname = hostname,
        .message = message,
    };
    const char *channel = getopt_get_string (state->gopt, "channel");
    perllcm_logbook_t_publish (state->lcm, channel, &entry);

    g_free (message);
    return 0;
}

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    state_t *state = calloc (1, sizeof (*state));
    state->gopt = getopt_create ();
    getopt_add_description (state->gopt, "logbook server/client utility.");
    getopt_add_help (state->gopt, NULL);

    getopt_add_string (state->gopt, 'c',  "channel", "LOGBOOK",  "LCM channel name");
    getopt_add_spacer (state->gopt, "---------------client opts--------------------");
    getopt_add_string (state->gopt, 'e',  "editor-cmd", "",  "use ARG as external editor (uses EDITOR environment variable o/w)");
    getopt_add_string (state->gopt, 'u',  "username",   "",  "specify a username ARG");
    getopt_add_string (state->gopt, 'm',  "message",    "",  "specify log message ARG");
    getopt_add_string (state->gopt, 'F',  "file",       "",  "read log message from file ARG");
    getopt_add_spacer (state->gopt, "---------------server opts--------------------");
    getopt_add_bool (state->gopt,   'S',  "server",     0,   "Run as server");
    getopt_add_bool (state->gopt,   'D',  "daemon",     0,   "Run as system daemon");
    getopt_add_bool (state->gopt,   'L',  "longfmt",    0,   "SVN style LONG log format");
    getopt_add_int  (state->gopt,   'n',  "new",       "0",  "Start a new log file every ARG seconds");
    getopt_add_bool (state->gopt,   'f',  "force",      0,   "Overwrite existing log file");
    getopt_add_bool (state->gopt,   'a',  "append",     0,   "Append to existing log file");
    getopt_add_bool (state->gopt,   'v',  "verbose",    0,   "Print log messages to screen");

    getopt_add_example (state->gopt,
                        "Client: send log message using default external editor\n"
                        "%s", argv[0]);
    getopt_add_example (state->gopt,
                        "Client: send log message using specified external editor and username\n"
                        "%s --username ryan --editor-cmd=\"emacs -nw\"", argv[0]);
    getopt_add_example (state->gopt,
                        "Client: send log message from command line\n"
                        "%s --message \"test 1 2 3\"", argv[0]);
    getopt_add_example (state->gopt,
                        "Client: send log message from file\n"
                        "%s -F message.txt", argv[0]);
    getopt_add_example (state->gopt,
                        "Server: log using default file name structure\n"
                        "%s -S", argv[0]);
    getopt_add_example (state->gopt,
                        "Server: log using specified file name structure with year, month, day using timeutil_strftime() syntax\n"
                        "%s -S MYLOG-%%Y-%%m-%%d", argv[0]);
    getopt_add_example (state->gopt,
                        "Server: start a new log file hourly\n"
                        "%s -S -n 3600", argv[0]);
    getopt_add_example (state->gopt,
                        "Server: use the SVN style LONG file format for entries (SHORT mode is default)\n"
                        "%s -S -longfmt", argv[0]);


    if (!getopt_parse (state->gopt, argc, argv, 1) || (state->gopt->extraargs->len > 1))
    {
        getopt_do_usage (state->gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (state->gopt, "help"))
    {
        getopt_do_usage (state->gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    // set external editor
    if (getopt_has_flag (state->gopt, "editor-cmd"))
        snprintf (state->editor, sizeof state->editor, "%s", getopt_get_string (state->gopt, "editor-cmd"));
    else if (g_getenv ("EDITOR")!=NULL)
        snprintf (state->editor, sizeof state->editor, "%s", g_getenv ("EDITOR"));
    else
        snprintf (state->editor, sizeof state->editor, "%s", DEFAULT_EDITOR);

    // init lcm
    state->lcm = lcm_create (NULL);
    if (!state->lcm)
    {
        ERROR ("lcm_create() failed");
        exit (EXIT_FAILURE);
    }

    // install custom signal handler
    struct sigaction act =
    {
        .sa_handler = my_signal_handler,
    };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);


    // act appropriately
    int ret = 0;
    if (getopt_get_bool (state->gopt, "server"))
        ret = server (state);
    else
        ret = client (state);

    // clean up
    getopt_destroy (state->gopt);
    lcm_destroy (state->lcm);
    g_free (state);

    return ret;
}
