#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h> // isdigit()
#include <inttypes.h> // needed for PRId64 macros
#include <libgen.h>
#include <limits.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include <pthread.h>
#include <glib.h>

#include "perls-lcmtypes/bot_core_image_t.h"

#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/unix.h"

#include "perls-vision/botimage.h"

typedef struct options options_t;
struct options
{
    char         *outdir;           // where?
    const char   *prefix;           // Prefix output image files with this
    bool         force;             // overwrite filenames
    bool         eightbit;          // convert 16-bit to 8-bit
    unsigned int compression;       // TIFF compression scheme
    unsigned int quality;           // 0-100 compression quality, for use with jpeg compression
    bool         nobayerfilt;       // if bayer image, just write raw bayer pattern w/o interpolation
    bool         quiet;             // don't be noisy
};

int
convert (options_t *opts, const char *ifile, const char *ofile)
{
    bot_core_image_t *ibot=NULL, *bayer=NULL, *eight=NULL;
    char *description = NULL, *channel = NULL;
    if (0 != vis_botimage_read_tiff (&ibot, &channel, &description, ifile, 0))
    {
        ERROR ("unable to read file %s", ifile);
        goto on_error;
    }

    if (opts->nobayerfilt)
        bayer = bot_core_image_t_copy (ibot);
    else
        vis_botimage_bayerfilt (&bayer, ibot);

    if (opts->eightbit)
        vis_botimage_16_to_8 (&eight, bayer);
    else
        eight = bot_core_image_t_copy (bayer);

    if (0 != vis_botimage_write_tiff (eight, ofile, channel, description, opts->compression))
    {
        ERROR ("unable to write file %s", ofile);
        goto on_error;
    }

    bot_core_image_t_destroy (ibot);
    bot_core_image_t_destroy (bayer);
    bot_core_image_t_destroy (eight);
    free (description);
    free (channel);

    return 0;

on_error:
    if (ibot)
        bot_core_image_t_destroy (ibot);
    if (bayer)
        bot_core_image_t_destroy (bayer);
    if (eight)
        bot_core_image_t_destroy (eight);
    if (description)
        free (description);
    if (channel)
        free (channel);
    return -1;
}

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // add options
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt,  "Bayer convert and/or compress cam-logger TIFF images.");
    getopt_add_string (gopt, 'c',  "compression",  "none",             "TIFF file compression scheme {none,lzw,deflate,jpeg,jpeg:quality}");
    getopt_add_bool   (gopt, '8',  "8bit",         0,                  "Convert to 8-bits of intensity.");
    getopt_add_bool   (gopt, 'f',  "force",        0,                  "Force overwrite of existing filename");
    getopt_add_bool   (gopt, 'n',  "nobayerfilt",  0,                  "Do not perform bayer color interpolation");
    getopt_add_string (gopt, 'o',  "outdir",       "./",               "Output directory");
    getopt_add_string (gopt, 'p',  "prefix",       "",                 "Output filename prefix");
    getopt_add_bool   (gopt, 'q',  "quiet",        0,                  "Don't print to screen");
    getopt_add_help   (gopt, NULL);
    getopt_add_example (gopt, "%s *.tif --outdir ~/foo/bar/.", argv[0]);

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len < 1)
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help"))
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    // parse options
    options_t *opts = g_malloc0 (sizeof (*opts));
    const char *compression_str = getopt_get_string (gopt, "compression");
    unsigned int quality = 95;
    if (0 == strcasecmp (compression_str, "none"))
        opts->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_NONE;
    else if (0 == strcasecmp (compression_str, "lzw"))
        opts->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_LZW;
    else if (0 == strcasecmp (compression_str, "deflate"))
        opts->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_DEFLATE;
    else if (0 == strcasecmp (compression_str, "jpeg"))
        opts->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_JPEG | quality;
    else if (0 == strncasecmp (compression_str, "jpeg:", 5) &&
             1 == sscanf (compression_str, "jpeg:%u", &quality) &&
             0 < quality && quality <= 100)
        opts->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_JPEG | quality;
    else
    {
        fprintf (stderr, "unrecognized compression format\n");
        exit (EXIT_FAILURE);
    }
    opts->eightbit = getopt_get_bool (gopt, "8bit");
    opts->force = getopt_get_bool (gopt, "force");
    opts->nobayerfilt = getopt_get_bool (gopt, "nobayerfilt");

    const char *outdir = getopt_get_string (gopt, "outdir");
    if (outdir[strlen (outdir)-1] != '/' &&
            outdir[strlen (outdir)-1] != '.')
    {
        ERROR ("[%s] does not specify a fully qualified directory path", outdir);
        exit (EXIT_FAILURE);
    }
    else
        opts->outdir = g_path_get_dirname (outdir);
    if (!g_file_test (opts->outdir, G_FILE_TEST_EXISTS))
        unix_mkpath (opts->outdir, 0775);


    opts->prefix = getopt_get_string (gopt, "prefix");
    opts->quiet = getopt_get_bool (gopt, "quiet");

    // convert images
    for (int i=0; i<gopt->extraargs->len; i++)
    {
        const char *ifullfile = gopt->extraargs->pdata[i];
        char *ifile = g_path_get_basename (ifullfile);
        char ofile[NAME_MAX], ofullfile[PATH_MAX];
        snprintf (ofile, sizeof ofile, "%s%s", opts->prefix, ifile);
        snprintf (ofullfile, sizeof ofullfile, "%s/%s", opts->outdir, ifile);
        if (g_file_test (ofullfile, G_FILE_TEST_EXISTS) && !opts->force)
        {
            printf ("file already exists, not overwriting: %s\n", ofullfile);
            free (ifile);
            continue;
        }

        if (!opts->quiet)
            printf ("Converting %s -> %s\n", ifullfile, ofullfile);
        convert (opts, ifullfile, ofullfile);
        free (ifile);
    }

    // clean up
    free (opts->outdir);
    free (opts);

    return 0;
}
