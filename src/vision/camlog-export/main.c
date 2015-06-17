#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h> // isdigit()
#include <inttypes.h>
#include <libgen.h>
#include <limits.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include "perls-lcmtypes/bot_core_image_t.h"

#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"
#include "perls-common/unix.h"

#include "perls-vision/botimage.h"

/* logging opts */
typedef struct opts opts_t;
struct opts
{
    char       channel[LCM_MAX_CHANNEL_NAME_LENGTH];
    char       logdir[PATH_MAX];  // where?
    int        compression;       // TIFF compression scheme
    uint8_t    quality;           // 0-100 compression quality, for use with jpeg compression
    char       strftime[1024];    // image filename format
    int        eight;             // 16 to 8 bit
    int        bayer;             // if color camera, log just the raw bayer pattern w/o interpolation
    bool       quiet;
};

static opts_t
process_options (getopt_t *gopt, int argc, char *argv[])
{
    opts_t opt =
    {
        .quality = 95,
    };

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len!=1)
    {
        getopt_do_usage (gopt, "LCMLOG");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help"))
    {
        getopt_do_usage (gopt, "LCMLOG");
        exit (EXIT_SUCCESS);
    }

    // channel
    strncpy (opt.channel, getopt_get_string (gopt, "channel"), sizeof opt.channel);

    // quiet
    opt.quiet = getopt_get_bool (gopt, "quiet");

    // logdir
    strncpy (opt.logdir, getopt_get_string (gopt, "logdir"), sizeof opt.logdir);

    // compression
    const char *compression = getopt_get_string (gopt, "compression");
    if (0==strcasecmp (compression, "none"))
        opt.compression = VIS_BOTIMAGE_TIFF_COMPRESSION_NONE;
    else if (0==strcasecmp (compression, "jpeg"))
        opt.compression = VIS_BOTIMAGE_TIFF_COMPRESSION_JPEG;
    else if (0==strncasecmp (compression, "jpeg:", 5))
    {
        opt.compression = VIS_BOTIMAGE_TIFF_COMPRESSION_JPEG;
        unsigned int q;
        if (1==sscanf (compression, "jpeg:%u", &q))
            opt.quality = q;
        else
            ERROR ("jpeg quality not parseable");
    }
    else if (0==strcasecmp (compression, "lzw"))
        opt.compression = VIS_BOTIMAGE_TIFF_COMPRESSION_LZW;
    else if (0==strcasecmp (compression, "deflate"))
        opt.compression = VIS_BOTIMAGE_TIFF_COMPRESSION_DEFLATE;
    else
    {
        ERROR ("unrecognized argument to --compression");
        exit (EXIT_FAILURE);
    }

    // strftime
    strncpy (opt.strftime, getopt_get_string (gopt, "strftime"), sizeof opt.strftime);

    // 8-bit
    opt.eight = getopt_get_bool (gopt, "8bit");

    // bayer
    opt.bayer = getopt_get_bool (gopt, "bayer");

    return opt;
}

void
bot_core_image_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                          const bot_core_image_t *msg, void *user)
{
    opts_t *opt = user;

    char filename[NAME_MAX];
    static uint32_t framecount = 0;
    vis_botimage_filename (filename, sizeof filename, opt->strftime, msg->utime, framecount++);
    if (!opt->quiet)
        printf("%s\r\n", filename);

    char logdir[PATH_MAX];
    snprintf (logdir, sizeof logdir, "%s/%s", opt->logdir, channel);
    unix_mkpath (logdir, 0775);

    bot_core_image_t *bot8;
    if (opt->eight)
    {
        vis_botimage_16_to_8 (&bot8, msg);
    }
    else
        bot8 = (bot_core_image_t *) msg;

    bot_core_image_t *botc;
    if (opt->bayer)
    {
        vis_botimage_bayerfilt (&botc, bot8);
    }
    else
        botc = (bot_core_image_t *) bot8;

    // write image to disk
    char fullname[PATH_MAX];
    snprintf (fullname, sizeof fullname, "%s/%s", logdir, filename);
    if (0 != vis_botimage_write_tiff (botc, fullname, channel, NULL, opt->compression | opt->quality))
        fprintf (stderr, "error writing: %s\n", filename);

    // clean up
    if (opt->bayer)
        bot_core_image_t_destroy (botc);
    if (opt->eight)
        bot_core_image_t_destroy (bot8);
}



int
main (int argc, char *argv[])
{
    // add options
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, "Extract images from lcmlog and write to disk");
    getopt_add_help   (gopt, NULL);
    getopt_add_string (gopt, 'c',  "channel",     "PROSILICA_C",      "LCM image channel name");
    getopt_add_string (gopt, 'd',  "logdir",      "./",               "Image directory");
    getopt_add_string (gopt, '\0', "compression", "none",             ".tif compression scheme {none,lzw,jpeg,jpeg:xx,deflate}");
    getopt_add_string (gopt, '\0', "strftime",    "%s%i.tif",         "Image file name strftime format, %f for frame number, %i for usecs");
    getopt_add_bool   (gopt, '8',  "8bit",        0,                  "Convert 16bit to 8bit?");
    getopt_add_bool   (gopt, '\0', "bayer",       0,                  "Export raw Bayer pattern?");
    getopt_add_bool   (gopt, 'q',  "quiet",       0,                  "Don't print to screen");

    opts_t opt = process_options (gopt, argc, argv);

    // setup lcm log file for playback
    const char *lcmlog_fullname = g_ptr_array_index (gopt->extraargs, 0);
    char *lcmlog_dir = dirname (strdup (lcmlog_fullname));
    char *lcmlog_fname = basename (strdup (lcmlog_fullname));

    char lcm_url[PATH_MAX];
    snprintf (lcm_url, sizeof lcm_url, "file://%s/%s?speed=0", lcmlog_dir, lcmlog_fname);
    lcm_t *lcm = lcm_create (lcm_url);
    if (!lcm)
        exit (EXIT_FAILURE);

    bot_core_image_t_subscribe (lcm, opt.channel, &bot_core_image_t_handler, &opt);
    while (0==lcm_handle (lcm)) {};

    lcm_destroy (lcm);
    free (lcmlog_dir);
    free (lcmlog_fname);

    return 0;

}
