#include <stdio.h>
#include <stdlib.h>

#include <bot_lcmgl_render/lcmgl_decode.h>

#include "perls-lcmtypes/bot_lcmgl_data_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/timestamp.h"

#include "renderers.h"

#define RENDERER_NAME "LCMGL"

typedef struct _RendererLCMGL RendererLCMGL;
struct _RendererLCMGL {
    BotRenderer renderer;

    BotViewer         *viewer;
    BotGtkParamWidget *pw;
    
    lcm_t *lcm;
    
    bot_lcmgl_data_t *last_lcmgl;
    
    char *channel;
    
    // bot config
    BotParam *param;   
};


static void
bot_lcmgl_data_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                     const bot_lcmgl_data_t *msg, void *user)
{
    RendererLCMGL *self = (RendererLCMGL*) user;
    
    if (NULL != self->last_lcmgl)
        bot_lcmgl_data_t_destroy (self->last_lcmgl);
        
    self->last_lcmgl = bot_lcmgl_data_t_copy (msg);
    bot_viewer_request_redraw (self->viewer);
}

static void
_renderer_free (BotRenderer *super)
{
    RendererLCMGL *self = (RendererLCMGL*) super->user;
    
    free (self);
}


static void 
lcmgl_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
    RendererLCMGL *self = (RendererLCMGL*) super->user;
    
    glEnable(GL_DEPTH_TEST);
    
    if (self->last_lcmgl)
        bot_lcmgl_decode (self->last_lcmgl->data, self->last_lcmgl->datalen);
}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererLCMGL *self = (RendererLCMGL*) user;
    bot_viewer_request_redraw (self->viewer);
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererLCMGL *self = (RendererLCMGL*) user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererLCMGL *self = (RendererLCMGL*) user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

static RendererLCMGL *
renderer_lcmgl_new (BotViewer *viewer, char *rootkey)
{
    RendererLCMGL *self = (RendererLCMGL*) calloc (1, sizeof (*self));
    
    self->lcm = bot_lcm_get_global (NULL);

    // load config -------------------------------------------------------------
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);

    char key[256] = {0};
    snprintf (key, sizeof key, "viewer.renderers.%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);
   
    snprintf (key, sizeof key, "viewer.renderers.%s.channel", rootkey);
    char *lcm_channel = bot_param_get_str_or_fail (self->param, key);
    bot_lcmgl_data_t_subscribe (self->lcm, lcm_channel, &bot_lcmgl_data_t_cb, self);
           
    //--------------------------------------------------------------------------
    BotRenderer *renderer = &self->renderer;

    renderer->draw = lcmgl_renderer_draw;
    renderer->destroy = _renderer_free;

    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = renderer_label;
    renderer->user = self;
    renderer->enabled = 1;
    
    self->viewer = viewer;
    
    self->last_lcmgl = NULL;
    
    self->pw = BOT_GTK_PARAM_WIDGET (renderer->widget);
    // GTK WIDGETS
    
    gtk_widget_show_all (renderer->widget);

    g_signal_connect (G_OBJECT (self->pw), "changed", 
                      G_CALLBACK (on_param_widget_changed), self);
    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
                      G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
                      G_CALLBACK (on_save_preferences), self);

    return self;
}


void
setup_renderer_lcmgl (BotViewer *viewer, char *rootkey, int render_priority)
{
    RendererLCMGL *self = renderer_lcmgl_new (viewer, rootkey);
    bot_viewer_add_renderer (viewer, &(self->renderer), render_priority);
}
