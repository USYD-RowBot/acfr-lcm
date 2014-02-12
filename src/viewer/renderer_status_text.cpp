#include <stdio.h>
#include <stdlib.h>

#include "perls-lcmtypes/perllcm_logbook_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/timestamp.h"

#include "renderers.h"

#define RENDERER_NAME "Status Text"

#define LAMBDA 0.1 // fade rate

typedef struct _RendererStatusText RendererStatusText;
struct _RendererStatusText {
    BotRenderer renderer;

    BotViewer         *viewer;
    BotGtkParamWidget *pw;
    
    BotEventHandler ehandler;
    
    lcm_t *lcm;
    
    int using_logbook;
    char *logbook_channel;
    
    BotGlConsole *console;
    
    // bot config
    BotParam *param;   
};

static void
_renderer_free (BotRenderer *super)
{
    RendererStatusText *self = (RendererStatusText*) super->user;
    
    if (self->console) 
        bot_gl_console_destroy (self->console);
    
    free (self);
}


static void 
status_text_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
    RendererStatusText *self = (RendererStatusText*) super->user;

    bot_gl_console_render (self->console, -1);
}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererStatusText *self = (RendererStatusText*) user;
    bot_viewer_request_redraw (self->viewer);
}

static void
perllcm_logbook_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                    const perllcm_logbook_t *msg, void *user)
{
    RendererStatusText *self = (RendererStatusText*) user;
    
    bot_gl_console_printf (self->console, "%ld %s@%s: %s  \n",
                           msg->utime,
                           msg->username,
                           msg->hostname,
                           msg->message);
    
    bot_viewer_request_redraw (self->viewer);
}

static void
send_logbook_msg (RendererStatusText *self)
{    
    GtkWidget *dialog = gtk_dialog_new_with_buttons ("Send Logbook Message",
                                                     NULL,
                                                     (GtkDialogFlags) (GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT),
                                                     GTK_STOCK_OK,
                                                     GTK_RESPONSE_ACCEPT,
                                                     GTK_STOCK_CANCEL,
                                                     GTK_RESPONSE_REJECT,
                                                     NULL);
    
    // get the content area
    GtkWidget *content = gtk_dialog_get_content_area (GTK_DIALOG (dialog));
    
    // add some widgets to the dialog
    // username
    gtk_box_pack_start (GTK_BOX (content), gtk_label_new ("Username:"), FALSE, FALSE, 0); 
    char username[128];
    snprintf (username, sizeof username, "%s", g_get_user_name ());
    GtkWidget *user_entry = gtk_entry_new ();
    gtk_entry_set_text (GTK_ENTRY (user_entry), username);
    gtk_entry_set_width_chars (GTK_ENTRY (user_entry), 20);
    gtk_box_pack_start (GTK_BOX (content), user_entry, FALSE, FALSE, 0);
    
    // host name
    gtk_box_pack_start (GTK_BOX (content), gtk_label_new ("Hostname:"), FALSE, FALSE, 0); 
    char hostname[256];
    snprintf (hostname, sizeof hostname, "%s", g_get_host_name ());
    GtkWidget *host_entry = gtk_entry_new ();
    gtk_entry_set_text (GTK_ENTRY (host_entry), hostname);
    gtk_entry_set_width_chars (GTK_ENTRY (host_entry), 20);
    gtk_box_pack_start (GTK_BOX (content), host_entry, FALSE, FALSE, 0);
    
    // message
    gtk_box_pack_start (GTK_BOX (content), gtk_label_new ("Message:"), FALSE, FALSE, 0); 
    GtkWidget *msg_entry = gtk_entry_new ();
    gtk_entry_set_text (GTK_ENTRY (msg_entry), "");
    gtk_entry_set_width_chars (GTK_ENTRY (msg_entry), 50);
    gtk_box_pack_start (GTK_BOX (content), msg_entry, FALSE, FALSE, 0);

    gtk_widget_show_all (dialog);
    
    if (gtk_dialog_run (GTK_DIALOG (dialog)) == GTK_RESPONSE_ACCEPT) {
        perllcm_logbook_t *entry = (perllcm_logbook_t*) calloc (1, sizeof (*entry));
        
        entry->utime = timestamp_now ();
        entry->username = strdup (gtk_entry_get_text (GTK_ENTRY (user_entry)));
        entry->hostname = strdup (gtk_entry_get_text (GTK_ENTRY (host_entry)));
        entry->message  = strdup (gtk_entry_get_text (GTK_ENTRY (msg_entry)));
        
        perllcm_logbook_t_publish (self->lcm, self->logbook_channel, entry);
        perllcm_logbook_t_destroy (entry);
    }

    gtk_widget_destroy (dialog);
}

static int 
on_key_press (BotViewer *viewer, BotEventHandler *ehandler, const GdkEventKey *event)
{
    RendererStatusText *self = (RendererStatusText*) ehandler->user;

    int keyval = event->keyval;
    
    switch (keyval) {
    case 'L':
    case 'l':
        if (event->state & GDK_CONTROL_MASK) { // holding ctrl -> ctrl+l
            if (self->using_logbook)
                send_logbook_msg (self);
        } 
        break;
    default:
        return 0;
    }

    return 1;
}


static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererStatusText *self = (RendererStatusText*) user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererStatusText *self = (RendererStatusText*) user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

static RendererStatusText *
renderer_status_text_new (BotViewer *viewer, char *rootkey)
{
    RendererStatusText *self = (RendererStatusText*) calloc (1, sizeof (*self));

    // load config -------------------------------------------------------------
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);

    char key[256] = {0};
    snprintf (key, sizeof key, "viewer.renderers.%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);
    
    self->lcm = bot_lcm_get_global (NULL);
   
    // load data sources
    snprintf (key, sizeof key, "viewer.renderers.%s.data_sources", rootkey);
    int num_data_sources = bot_param_get_num_subkeys (self->param, key);
    printf ("\tNumber of Status Text data sources %d \n", num_data_sources);
    char **data_source_keys;
    data_source_keys = bot_param_get_subkeys (self->param, key);
    // loop over each data source
    for (int i=0; i<num_data_sources; i++) {
        snprintf (key, sizeof key, "viewer.renderers.%s.data_sources.%s", rootkey, data_source_keys[i]);
        char *lcm_channel = bot_param_get_str_or_fail (self->param, key);
        if (0==strcmp ("perllcm_logbook", data_source_keys[i])) {
            perllcm_logbook_t_subscribe (self->lcm, lcm_channel, &perllcm_logbook_cb, self);
            self->using_logbook = 1;
            self->logbook_channel = lcm_channel;
        }
        else {
            ERROR ("\tData source %s not found for %s \n", data_source_keys[i], rootkey);
            continue;
        }
        printf ("\tAdded data source: %s \n", data_source_keys[i]);
    }
        
    //--------------------------------------------------------------------------
    BotRenderer *renderer = &self->renderer;

    renderer->draw = status_text_renderer_draw;
    renderer->destroy = _renderer_free;

    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = renderer_label;
    renderer->user = self;
    renderer->enabled = 1;

    // setup event callbacks
    BotEventHandler *ehandler = &self->ehandler;
    memset (ehandler, 0x00, sizeof (*ehandler));
    ehandler->name = (char*) RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->mouse_press = NULL;
    ehandler->mouse_motion = NULL;
    ehandler->mouse_release = NULL;
    ehandler->mouse_scroll = NULL;
    ehandler->pick_query = NULL;
    ehandler->hover_query = NULL;
    ehandler->key_press = on_key_press;
    ehandler->user = self;

    self->viewer = viewer;
    
    self->console = bot_gl_console_new ();
    bot_gl_console_color3f (self->console, 0.1, 0.1, 0.1);
    bot_gl_console_set_decay_lambda (self->console, LAMBDA);

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
setup_renderer_status_text (BotViewer *viewer, char *rootkey, int render_priority)
{
    RendererStatusText *self = renderer_status_text_new (viewer, rootkey);
    bot_viewer_add_renderer (viewer, &(self->renderer), render_priority);
    bot_viewer_add_event_handler (viewer, &(self->ehandler), render_priority);
}
