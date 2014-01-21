#include <stdio.h>
#include <stdlib.h>
#include <glib.h>

#include "perls-common/textread.h"

#include "lcmlog_export.h"

struct lcmlog_export {
    // output CSV related
    char *csvlog_dir;
    char *csvlog_prefix;

    // LCM channel related
    char *channel_strip;
    
    char *delim_string;

    GHashTable *hash;
};

static void
wrapper_textread_destroy (gpointer data)
{
    textread_destroy (data);
}

lcmlog_export_t *
lle_create (const char *csvlog_dir, const char *csvlog_prefix, const char *channel_strip, 
            const char *delim_string)
{
    lcmlog_export_t *lle = calloc (1, sizeof (*lle));
    lle->csvlog_dir = csvlog_dir ? strdup (csvlog_dir) : strdup ("./");
    lle->csvlog_prefix = csvlog_prefix ? strdup (csvlog_prefix) : strdup ("LCMLOG_EXPORT");
    lle->channel_strip = channel_strip ? strdup (channel_strip) : NULL;
    lle->delim_string = delim_string ? strdup (delim_string) : NULL;
    lle->hash = g_hash_table_new_full (&g_str_hash, &g_str_equal, &g_free, &wrapper_textread_destroy);
    return lle;
}

void
lle_destroy (lcmlog_export_t *lle)
{
    g_free (lle->csvlog_dir);
    g_free (lle->csvlog_prefix);
    g_free (lle->channel_strip);
    g_free (lle->delim_string);
    g_hash_table_remove_all (lle->hash);
}

textread_t *
lle_get_textread (lcmlog_export_t *lle, const char *_channel)
{
    bool strip = false;
    const char *channel = strdup (_channel);

    // strip | separated list of channel prefixes
    if (lle->channel_strip) {
        char **split = g_strsplit (lle->channel_strip, "|", -1);
        for (char **prefix=split; *prefix; prefix++) {
            if (g_str_has_prefix (_channel, *prefix)) {
                strip = true;
                channel += strlen (*prefix);
                break;
            }
        }
        g_strfreev (split);
    }

    // change all delimiter strings to underscores
    if (lle->delim_string) 
        channel = g_strdelimit ((gchar *) channel, lle->delim_string, '_');   

    textread_t *tr = g_hash_table_lookup (lle->hash, channel);
    if (!tr) {
        if (strip)
            printf ("export %-32s\t[%s]\n", channel, _channel);
        else
            printf ("export %-32s\n", channel);
        tr = textread_create (lle->csvlog_dir, lle->csvlog_prefix, channel);
        g_hash_table_insert (lle->hash, strdup (channel), tr);
    }
    return tr;
}

textread_t **
lle_get_textread_array (lcmlog_export_t *lle)
{
    textread_t **tra = calloc (lle_get_length (lle), sizeof (textread_t *));
    GHashTableIter iter;
    gpointer key, value;
    g_hash_table_iter_init (&iter, lle->hash);
    for (int i=0; g_hash_table_iter_next (&iter, &key, &value); i++)
        tra[i]= value;

    return tra;
}

size_t
lle_get_length (lcmlog_export_t *lle)
{
    return g_hash_table_size (lle->hash);
}

const char *
lle_get_csvdir (lcmlog_export_t *lle)
{
    return lle->csvlog_dir;
}
