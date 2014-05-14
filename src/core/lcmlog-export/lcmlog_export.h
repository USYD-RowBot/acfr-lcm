#ifndef __LCMLOG_EXPORT_H__
#define __LCMLOG_EXPORT_H__

#include "perls-common/textread.h"

typedef struct lcmlog_export lcmlog_export_t;

lcmlog_export_t *
lle_create (const char *csvlog_dir, const char *csvlog_prefix, const char *channel_strip, 
            const char *delim_string);

void
lle_destroy (lcmlog_export_t *lle);

textread_t *
lle_get_textread (lcmlog_export_t *lle, const char *channel);

textread_t **
lle_get_textread_array (lcmlog_export_t *lle);

size_t
lle_get_length (lcmlog_export_t *lle);

const char *
lle_get_csvdir (lcmlog_export_t *lle);

#endif //__LCMLOG_EXPORT_H__
