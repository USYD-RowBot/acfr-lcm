#include <cstdio>
#include <cstdlib>
#include <cstring>


#include "perls-common/units.h"
#include "perls-common/bot_util.h"

#include "perls-lcmtypes/hauv_bs_pit_t.h"
#include "perls-lcmtypes/senlcm_prosilica_t.h"

#include "renderers.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define PARAM_MANUAL_OVERRIDE "Enable Manual Override"

#define HAUV_PIT_CHANNEL         "HAUV_BS_PIT"
#define PITCH_SONAR_UPPER_LIM  0.5*DTOR
#define PITCH_SONAR_LOWER_LIM -0.5*DTOR

typedef struct _RendererCamctrl RendererCamctrl;

struct _RendererCamctrl {

    //bot gui stuff
    BotRenderer renderer;
    BotViewer *viewer;
    BotGtkParamWidget  *pw;

    BotParam *param;

    lcm_t *lcm;

    char *camChannel;

    double pitch_sonar;
    bool startCapture;
    bool manualOverride;
};

//Return true if the angle is in the range of angles where the camera should trigger
static bool
shouldCapture(double angle)
{
    return (angle < PITCH_SONAR_UPPER_LIM) && (angle > PITCH_SONAR_LOWER_LIM);
}

static void
start_capturing (RendererCamctrl *self)
{
    //Check that the dvl pitch is within range
    if (!self->manualOverride && !shouldCapture(self->pitch_sonar))
        return;

    senlcm_prosilica_attribute_t *attrib = (senlcm_prosilica_attribute_t *)calloc(1, sizeof(attrib));
    attrib->label = strdup("AcquisitionStart");
    attrib->value = strdup("True");

    senlcm_prosilica_t *camMsg = (senlcm_prosilica_t *)calloc(1, sizeof(camMsg));
    camMsg->n_attributes = 1;
    camMsg->PvAttributes = attrib;

    senlcm_prosilica_t_publish (self->lcm, self->camChannel, camMsg);

    //clean up
    free(attrib->label);
    free(attrib->value);
    free(attrib);
    free(camMsg);
}

static void
stop_capturing (RendererCamctrl *self)
{
    if (!self->manualOverride && shouldCapture(self->pitch_sonar))
        return;

    senlcm_prosilica_attribute_t *attrib = (senlcm_prosilica_attribute_t *)calloc(1, sizeof(attrib));
    attrib->label = strdup("AcquisitionStop");
    attrib->value = strdup("True");

    senlcm_prosilica_t *camMsg = (senlcm_prosilica_t *)calloc(1, sizeof(camMsg));
    camMsg->n_attributes = 1;
    camMsg->PvAttributes = attrib;

    senlcm_prosilica_t_publish (self->lcm, self->camChannel, camMsg);

    //clean up
    free(attrib->label);
    free(attrib->value);
    free(attrib);
    free(camMsg);
}

static void
decideWhatToDo(RendererCamctrl *self)
{
    if (self->startCapture)
        start_capturing(self);
    else
        stop_capturing(self);
}

static void
on_start_capture (GtkWidget *button, void *user)
{
    RendererCamctrl *self = (RendererCamctrl *)user;
    self->startCapture = true;
    decideWhatToDo(self);
}

static void
on_stop_capture (GtkWidget *button, void *user)
{
    RendererCamctrl *self = (RendererCamctrl *)user;
    self->startCapture = false;
    decideWhatToDo(self);
}

static void
hauv_bs_pit_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                        const hauv_bs_pit_t *msg, void *user)
{
    RendererCamctrl *self = (RendererCamctrl *)user;
    self->pitch_sonar = msg->pitch_sonar;
    
    if (!self->manualOverride)
        self->startCapture = shouldCapture(self->pitch_sonar);

    decideWhatToDo(self);
}

static void
camctrl_draw (BotViewer *viewer, BotRenderer *renderer)
{
    //pass
}

static void
camctrl_free (BotRenderer *renderer) 
{
    if (!renderer)
        return;

    RendererCamctrl *self = (RendererCamctrl*) renderer->user;
 
    if (!self)
        return;

    free (self->camChannel);
    free (self);
}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererCamctrl *self = (RendererCamctrl *)user;
    self->manualOverride = bot_gtk_param_widget_get_bool (self->pw, PARAM_MANUAL_OVERRIDE);
}

static RendererCamctrl *
renderer_camctrl_new (BotViewer *viewer, char *rootkey)
{
    RendererCamctrl *self = new RendererCamctrl();
    self->manualOverride = false;
    
    //libbot-related
    BotRenderer *renderer = &self->renderer;
    renderer->draw = camctrl_draw;
    renderer->destroy = camctrl_free;
    renderer->widget = bot_gtk_param_widget_new ();
    renderer->user = self;
    renderer->enabled = 1;
    renderer->name = strdup("camctrl");
    self->pw = BOT_GTK_PARAM_WIDGET (renderer->widget);
    self->camChannel = NULL;

    //Create the BotParam
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    self->startCapture = false;
    self->pitch_sonar = 999999.0;

    if (bot_param_get_str (self->param, "prosilica.mono.channel", &self->camChannel) == 0) {
        char *tmp = (char *)malloc(128 * sizeof(char));
        snprintf(tmp, 128, "%s%s", self->camChannel, ".ATTRIBUTES");
        free(self->camChannel);
        self->camChannel = tmp;
    }
    else {
        self->camChannel = strdup("PROSILICA_M.ATTRIBUTES");
    }

    //LCM-related
    self->lcm = bot_lcm_get_global (NULL);
    hauv_bs_pit_t_subscribe (self->lcm, HAUV_PIT_CHANNEL, &hauv_bs_pit_t_callback, self);

    //GUI-related
    self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);
    GtkWidget *vbox = gtk_vbox_new (FALSE, 0);

    gtk_container_add (GTK_CONTAINER (self->renderer.widget), vbox);
    GtkWidget *start_capture_button = gtk_button_new_with_label ("Start Capture");
    GtkWidget *stop_capture_button = gtk_button_new_with_label ("Stop Capture");
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_MANUAL_OVERRIDE, 0, NULL);

    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET (self->pw),FALSE, TRUE, 0);
    gtk_box_pack_start (GTK_BOX (vbox), start_capture_button, FALSE, FALSE, 0);
    gtk_box_pack_start (GTK_BOX (vbox), stop_capture_button, FALSE, FALSE, 0);
    gtk_widget_show (GTK_WIDGET (self->pw));
    gtk_widget_show_all (self->renderer.widget);

    //GUI callbacks
    g_signal_connect (G_OBJECT (stop_capture_button), "clicked", G_CALLBACK (on_stop_capture), self);
    g_signal_connect (G_OBJECT (start_capture_button), "clicked", G_CALLBACK (on_start_capture), self);
    g_signal_connect (G_OBJECT (self->pw), "changed", G_CALLBACK (on_param_widget_changed), self);

    return self;
}

void
setup_renderer_camctrl (BotViewer *viewer, char *rootkey, int render_priority)
{
    RendererCamctrl *self = renderer_camctrl_new (viewer, rootkey);
    bot_viewer_add_renderer (viewer, &(self->renderer), render_priority);
}
