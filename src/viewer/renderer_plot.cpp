#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <deque>
#include <algorithm>

// external linking req'd
#include "perls-lcmtypes/perllcm_position_t.h"
#include "perls-lcmtypes/perllcm_lcmplot_t.h"
#include <gsl/gsl_math.h>

#include "perls-common/bot_util.h"
#include "perls-common/units.h"
#include "perls-common/timestamp.h"

#include "renderers.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define RENDERER_NAME "Plot"
#define TIME_HISTORY "Time History"
#define PARAM_CLEAR_HISTORY "Clear History"
#define PLOT_ENABLE "Enable"
#define PLOT_SHOW_LEGEND "Show Legend"
#define X_PCT "X Percent"
#define Y_PCT "Y Percent"
#define WIDTH_PCT "Width Percent"
#define HEIGHT_PCT "Height Percent"


#define POSITION_PLOT 1
#define LCM_PLOT 0

class window {
public:
    BotGlScrollPlot2d *p;               // bot plot type
    std::deque<double> time;
    std::deque<double> data_max;
    std::deque<double> data_min;
    std::vector<char*> series_label;
    std::vector<bool> series_enable;
    int max_datapts;

    window (int max_datapts)
    {
        this->p = bot_gl_scrollplot2d_new ();
        bot_gl_scrollplot2d_set_bgcolor (this->p, 1, 1, 1, 0.5);
        bot_gl_scrollplot2d_set_border_color (this->p, 0, 0, 0, 0.15);
        bot_gl_scrollplot2d_set_text_color (this->p, 0, 0, 0, 1);

        this->max_datapts = max_datapts;
    }

    ~window (void)
    {
        bot_gl_scrollplot2d_free (p);
        series_label.clear ();
    }


    int
    check_series (const char *name, bool en)
    {
        // check if series name already in plot
        int idx = -1;
        for (std::vector<char*>::iterator it = series_label.begin ();
                it !=  series_label.end(); ++it) {
            if (0 == strcmp (*it, name)) {
                idx = it - series_label.begin();
                return idx;
            }
        }

        // need to init plot type
        char *n = new char[256];
        snprintf (n, 256, "%s", name);
        series_label.push_back (n);
        series_enable.push_back (en);

        // init color, too
        idx = series_label.size()-1;
        int r = idx % 7;

        //000/001/010/011/100/101/110
        bot_gl_scrollplot2d_add_plot (this->p, name, max_datapts);
        bot_gl_scrollplot2d_set_color (this->p, name, (r>>2)&1, (r>>1)&1, r&1, 1);

        return idx;
    }

    void
    add_data_point (const char *name, int64_t utime, double val)
    {
        int idx = check_series (name, true);
        if (!this->series_enable[idx]) return;

        double time_seconds = double (utime) / 1e6;
        std::deque<double>::iterator iter = std::find (this->time.begin(), this->time.end(), time_seconds);
        if (iter == this->time.end ()) {
            // just add to set
            if (int(this->time.size ()) >= this->max_datapts) {
                this->time.pop_front ();
                this->data_max.pop_front ();
                this->data_min.pop_front ();
            }

            this->time.push_back (time_seconds);
            this->data_max.push_back (val);
            this->data_min.push_back (val);
        }
        else {
            // need to modify min/max values
            int idx = iter - this->time.begin();
            this->data_max[idx] = GSL_MAX_DBL (this->data_max[idx], val);
            this->data_min[idx] = GSL_MIN_DBL (this->data_min[idx], val);
        }
            
        // add to plot set
        bot_gl_scrollplot2d_add_point (p, name, time_seconds, val);
    }

    void
    render (double x, double y, double w, double h, bool leg, int history)
    {
        std::deque<double>::iterator min_time, max_time;
        std::deque<double>::iterator min_data, max_data;

        if (this->time.size() == 0 || history == 0) return;

        min_time = std::min_element(this->time.begin() + 
                GSL_MAX (int(this->time.size()) - history, 0), 
                this->time.end());
        max_time = std::max_element(this->time.begin() + 
                GSL_MAX (int(this->time.size()) - history, 0), 
                this->time.end());
        min_data = std::min_element(this->data_min.begin() + 
                GSL_MAX (int(this->data_min.size()) - history, 0), 
                this->data_min.end());
        max_data = std::max_element(this->data_max.begin() + 
                GSL_MAX (int(this->data_max.size()) - history, 0), 
                this->data_max.end());

        bot_gl_scrollplot2d_set_xlim (p, *min_time-.1, *max_time+.1);
        bot_gl_scrollplot2d_set_ylim (p, *min_data-.1, *max_data+.1);

        bot_gl_scrollplot2d_gl_render_at_window_pos (p, x, y, w, h);

        if (leg)
            bot_gl_scrollplot2d_set_show_legend (p, BOT_GL_SCROLLPLOT2D_TOP_RIGHT);
        else
            bot_gl_scrollplot2d_set_show_legend (p, BOT_GL_SCROLLPLOT2D_HIDDEN);
    }

    void
    set_title (const char *name)
    {
        bot_gl_scrollplot2d_set_title (p, name);
    }

    void
    clear ()
    {
        this->time.clear ();
        this->data_max.clear ();
        this->data_min.clear ();
    }
};

class plots {
public:
    std::vector<window*> windows;    // collection of sub windows
    int history;

    plots (int n_windows, int max_datapts)
    {
        for (int i=0; i<n_windows; ++i)
            this->windows.push_back (new window (max_datapts));
        history = 1;
    }

    ~plots (void)
    {
        this->windows.clear ();
    }

    int
    pre_add_series (int window_idx, const char *name, bool en)
    {
        return windows[window_idx]->check_series (name, en);
    }

    int 
    get_max_history ()
    {
        int m = -1;
        for (int i=0; i<int(this->windows.size ()); ++i) {
            int tmp = this->windows[i]->time.size ();
            if (tmp > m) m = tmp;
        }

        return m;
    }

    void
    add_data_point (BotGtkParamWidget *pw, int window_idx, const char *name, int64_t utime, double val)
    {
        // check max size
        int bef = this->get_max_history ();

        // add point
        windows[window_idx]->add_data_point (name, utime, val);

        // check max size again
        int aft = this->get_max_history ();

        // if max == history, increase history by one
        if (this->history == (bef+1) && aft > bef)
            this->history++;

        bot_gtk_param_widget_modify_int (pw, TIME_HISTORY, 0, aft+1, 1, this->history);
    }

    void
    render (double w_width, double w_height, int x_pct, int y_pct, int w_pct, int h_pct, bool legend)
    {
        double x, y, w, h;
        h = double(h_pct)/100 * w_height / double(this->windows.size());

        for (int i=0; i<int(this->windows.size()); ++i) {
            x = double(x_pct)/100 * w_width;
            y = double(y_pct)/100 * w_height + i*h;
            w = double(w_pct)/100 * w_width;

            this->windows[i]->render (x, y, w, h, legend, this->history);
        }

    }

    void
    set_title (int window_idx, const char *name)
    {
        this->windows[window_idx]->set_title (name);
    }

    void
    clear ()
    {
        this->history = 1;

        for (int i=0; i<int(this->windows.size ()); ++i)
            windows[i]->clear ();
    }

    void
    update_history (BotGtkParamWidget *pw)
    {
        history = bot_gtk_param_widget_get_int (pw, TIME_HISTORY);
    }
};

typedef struct _RendererPlot RendererPlot;
struct _RendererPlot {
    BotRenderer renderer;
    
    lcm_t *lcm;
    BotParam *param;
    
    BotViewer         *viewer;
    BotGtkParamWidget *pw;   

    int plot_type;

    int x_pct;
    int y_pct;
    int width_pct;
    int height_pct;

    bool enable;
    bool show_legend;

    plots *plot_set;
};


static void
position_cb (const lcm_recv_buf_t *rbuf, const char *channel,
          const perllcm_position_t *msg, void *user)
{    
    RendererPlot *self = (RendererPlot*) user;

    for (int i=0; i<6; ++i) {
        self->plot_set->add_data_point (self->pw, i, channel, msg->utime, msg->xyzrph[i]);
        double sig3 = 3*sqrt (msg->xyzrph_cov[7*i]);
        char name[256];

        snprintf (name, sizeof name, "%s.covup", channel);
        self->plot_set->add_data_point (self->pw, i, name, msg->utime, msg->xyzrph[i]+sig3);

        snprintf (name, sizeof name, "%s.covdown", channel);
        self->plot_set->add_data_point (self->pw, i, name, msg->utime, msg->xyzrph[i]-sig3);
    }
    
    bot_viewer_request_redraw (self->viewer);
}

static void
lcmplot_cb (const lcm_recv_buf_t *rbuf, const char *channel,
          const perllcm_lcmplot_t *msg, void *user)
{    
    RendererPlot *self = (RendererPlot*) user;

    // add each data point
    for (int i=0; i<msg->npts; ++i) {
        self->plot_set->add_data_point (self->pw, 0, msg->label[i], msg->utime, msg->pts[i]);
    }

    bot_viewer_request_redraw (self->viewer);
}

static void
_renderer_free (BotRenderer *super)
{
    RendererPlot *self = (RendererPlot*) super->user;

    delete self->plot_set;

    free (self);
}

static void 
plot_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
    RendererPlot *self = (RendererPlot*) super->user;
    glEnable (GL_DEPTH_TEST);

    if (!self->enable)
        return;

    // get window size
    int vp[4] = {0,0,0,0};
    if (viewer->gl_area && bot_gtk_gl_drawing_area_set_context (viewer->gl_area) == 0)
        glGetIntegerv(GL_VIEWPORT, vp);
    int window_width = vp[2], window_height = vp[3];

    self->plot_set->render (window_width, window_height, self->x_pct, self->y_pct, 
            self->width_pct, self->height_pct, self->show_legend);
}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererPlot *self = (RendererPlot*) user;

    if (0==strcmp (name, PLOT_ENABLE))
        self->enable = !self->enable;
    else if (0==strcmp (name, PLOT_SHOW_LEGEND))
        self->show_legend = !self->show_legend;
    else if (0==strcmp (name, X_PCT)) {
        self->x_pct = bot_gtk_param_widget_get_int (self->pw, X_PCT);
        self->width_pct = GSL_MIN (100-self->x_pct, self->width_pct);
        bot_gtk_param_widget_modify_int (self->pw, WIDTH_PCT, 
                        0, 100-self->x_pct, 1, self->width_pct);
    }
    else if (0==strcmp (name, Y_PCT)) {
        self->y_pct = bot_gtk_param_widget_get_int (self->pw, Y_PCT);
        self->height_pct = GSL_MIN (100-self->y_pct, self->height_pct);
        bot_gtk_param_widget_modify_int (self->pw, HEIGHT_PCT, 
                        0, 100-self->y_pct, 1, self->height_pct);
    }
    else if (0==strcmp (name, WIDTH_PCT)) {
        self->width_pct = bot_gtk_param_widget_get_int (self->pw, WIDTH_PCT);
    }
    else if (0==strcmp (name, HEIGHT_PCT)) {
        self->height_pct = bot_gtk_param_widget_get_int (self->pw, HEIGHT_PCT);
    }
    else if (0==strcmp (name, TIME_HISTORY)) {
        self->plot_set->update_history (self->pw);
    }

    bot_viewer_request_redraw (self->viewer);
}

static void 
on_clear_history (GtkWidget *button, void *user)
{
    RendererPlot *self = (RendererPlot*) user;
    
    self->plot_set->clear ();

    bot_viewer_request_redraw (self->viewer);
}


static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererPlot *self = (RendererPlot*) user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererPlot *self = (RendererPlot*) user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

static RendererPlot *
renderer_plot_new (BotViewer *viewer, char *rootkey)
{    
    RendererPlot *self = (RendererPlot*) calloc (1, sizeof (*self));
    
    self->lcm = bot_lcm_get_global (NULL);

    // load config -------------------------------------------------------------
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);

    char key[256] = {0};

    snprintf (key, sizeof key, "viewer.renderers.%s.plot_key", rootkey);
    char *plot_key = bot_param_get_str_or_fail (self->param, key);

    snprintf (key, sizeof key, "viewer.renderers.%s.max_datapts", rootkey);
    double max_datapts = GSL_MAX (GSL_MIN (botu_param_get_int_or_default (self->param, key, 1000), 1000000), 1);

    // INITIALIZE PLOTS
    // ** CHANGE THIS FOR OTHER TYPES **
    if (0==strcmp ("position", plot_key)) {
        self->plot_type = POSITION_PLOT;
        self->plot_set = new plots (6, max_datapts);
    }
    else if (0==strcmp ("lcmplot", plot_key)) {
        self->plot_type = LCM_PLOT;
        self->plot_set = new plots (1, max_datapts);
    }
    else {
        printf ("PLOT RENDERER: PLOT KEY NOT FOUND: %s \n", plot_key);
        abort ();
    }

    free (plot_key);

    snprintf (key, sizeof key, "viewer.renderers.%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);
   
    snprintf (key, sizeof key, "viewer.renderers.%s.init_x", rootkey);
    self->x_pct = GSL_MAX_DBL (GSL_MIN_DBL (botu_param_get_double_or_default (self->param, key, 0), 100), 0);

    snprintf (key, sizeof key, "viewer.renderers.%s.init_y", rootkey);
    self->y_pct = GSL_MAX_DBL (GSL_MIN_DBL (botu_param_get_double_or_default (self->param, key, 0), 100), 0);

    snprintf (key, sizeof key, "viewer.renderers.%s.width", rootkey);
    self->width_pct = GSL_MAX_DBL (GSL_MIN_DBL (botu_param_get_double_or_default (self->param, key, 0), 100-self->x_pct), 0);

    snprintf (key, sizeof key, "viewer.renderers.%s.height", rootkey);
    self->height_pct = GSL_MAX_DBL (GSL_MIN_DBL (botu_param_get_double_or_default (self->param, key, 0), 100-self->y_pct), 0);


    // read all series
    // ** CHANGE THIS FOR OTHER TYPES **
    snprintf (key, sizeof key, "viewer.renderers.%s", rootkey);
    int num_sub = bot_param_get_num_subkeys (self->param, key);
    char **sub_keys = bot_param_get_subkeys (self->param, key);
    char key2[512] = {0};
    for (int i=0; i<num_sub; i++) {
        snprintf (key2, sizeof key2, "%s.%s.channel", key, sub_keys[i]);
        if (bot_param_has_key (self->param, key2)) {
            char *channel = bot_param_get_str_or_fail (self->param, key2);
            int r;

            if (self->plot_type == POSITION_PLOT) {
                /* subscribe to series */
                perllcm_position_t_subscribe (self->lcm, channel, &position_cb, self);

                /* check for position enables */
                snprintf (key2, sizeof key2, "%s.%s.enable_pose", key, sub_keys[i]);
                int en[6], cov_en[6];
                r = bot_param_get_int_array (self->param, key2, en, 6);

                if (r < 6) 
                    for (int j=0; j<6; ++j) en[j] = 1;

                /* check for covariance enables */
                snprintf (key2, sizeof key2, "%s.%s.enable_cov", key, sub_keys[i]);
                r = bot_param_get_int_array (self->param, key2, cov_en, 6);

                if (r < 6)
                    for (int j=0; j<6; ++j) cov_en[j] = 1;

                for (int j=0; j<6; ++j) {
                    self->plot_set->pre_add_series (j, channel, en[j]);
                    char name[256];
                    snprintf (name, sizeof name, "%s.covup", channel);
                    self->plot_set->pre_add_series (j, name, cov_en[j]);

                    snprintf (name, sizeof name, "%s.covdown", channel);
                    self->plot_set->pre_add_series (j, name, cov_en[j]);
                }

                self->plot_set->set_title (0, "x");
                self->plot_set->set_title (1, "y");
                self->plot_set->set_title (2, "z");
                self->plot_set->set_title (3, "roll");
                self->plot_set->set_title (4, "pitch");
                self->plot_set->set_title (5, "yaw");
            }
            else if (self->plot_type == LCM_PLOT) {
                /* subscribe to series */
                perllcm_lcmplot_t_subscribe (self->lcm, channel, &lcmplot_cb, self);

                /* check for single bit enable */
                snprintf (key2, sizeof key2, "%s.%s.enable", key, sub_keys[i]);
                r = botu_param_get_int_or_default (self->param, key2, 1);
            }

            free (channel);
        }
    }

    //--------------------------------------------------------------------------

    /* build renderer */
    BotRenderer *renderer = &self->renderer;
    self->enable = true;
    self->show_legend = true;

    renderer->draw = plot_renderer_draw;
    renderer->destroy = _renderer_free;

    renderer->widget = bot_gtk_param_widget_new ();
    renderer->name = renderer_label;
    renderer->user = self;
    renderer->enabled = 1;

    self->viewer = viewer;

    /* set widgets */
    self->pw = BOT_GTK_PARAM_WIDGET (renderer->widget);

    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, 
                                PLOT_ENABLE, 1, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, 
                                PLOT_SHOW_LEGEND, 1, NULL);
    bot_gtk_param_widget_add_int (self->pw, TIME_HISTORY, BOT_GTK_PARAM_WIDGET_SLIDER, 
                                0, 1, 1, 1);
    // clear history btn
    GtkWidget *clear_history_button = gtk_button_new_with_label (PARAM_CLEAR_HISTORY);
    GtkWidget *vbox = gtk_vbox_new (FALSE, 0);
    gtk_container_add (GTK_CONTAINER (self->renderer.widget), vbox);
    gtk_widget_show (vbox);
    gtk_box_pack_start (GTK_BOX (vbox), clear_history_button, FALSE, FALSE, 0);
    g_signal_connect (G_OBJECT (clear_history_button), "clicked", G_CALLBACK (on_clear_history), self);

    bot_gtk_param_widget_add_int (self->pw, X_PCT, BOT_GTK_PARAM_WIDGET_SLIDER, 
                                0, 100, 1, self->x_pct);
    bot_gtk_param_widget_add_int (self->pw, Y_PCT, BOT_GTK_PARAM_WIDGET_SLIDER, 
                                0, 100, 1, self->y_pct);
    bot_gtk_param_widget_add_int (self->pw, WIDTH_PCT, BOT_GTK_PARAM_WIDGET_SLIDER, 
                                0, 100-self->x_pct, 1, self->width_pct);
    bot_gtk_param_widget_add_int (self->pw, HEIGHT_PCT, BOT_GTK_PARAM_WIDGET_SLIDER, 
                                0, 100-self->y_pct, 1, self->height_pct);

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
setup_renderer_plot (BotViewer *viewer, char *rootkey, int priority)
{
    RendererPlot *self = renderer_plot_new (viewer, rootkey);
    bot_viewer_add_renderer (viewer, &self->renderer, priority);
}
