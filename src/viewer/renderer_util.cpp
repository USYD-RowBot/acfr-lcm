#include <iostream>
#include "renderer_util.h"

float colors[] = {
    1.0, 0.0, 0.0, // red   0
    0.0, 1.0, 0.0, // lime  1
    0.0, 0.0, 1.0, // blue  2
    0.5, 0.0, 0.0,
    0.0, 0.5, 0.0, // green 4
    0.0, 0.0, 0.5,
    0.5, 0.5, 0.5, // gray  6
    1.0, 1.0, 0.0,
    1.0, 0.0, 1.0, // pink  8
    0.0, 1.0, 1.0  // cyan  9
};

typedef enum _line_style_t
{
    SOLID       = 1,
    DOTTED      = 2,
    THICK       = 3,
    DOT         = 4,
    THICK_DOT   = 5,
} line_style_t;

// Dynamic checkbox class
//---------------------------------------------
typedef struct {
    DynCheckbox* renu;
    int id;
} checkbox_info_t;

static void 
on_checkbox_changed (GtkWidget *box, checkbox_info_t* info)
{
    g_mutex_lock(info->renu->cbMutex);
        bool is_checked = gtk_toggle_button_get_active((GtkToggleButton*)box);
        info->renu->checkbox_on[info->id] = is_checked;
    g_mutex_unlock(info->renu->cbMutex);

    bot_viewer_request_redraw (info->renu->viewer);
}

void 
DynCheckbox::add_checkbox (const char* name, int sensor_id, int type) 
{
    int id = sensor_id + type;

    if (checkbox_exist[id])
        return;

    int color_idx = m_lineprop->get_color_idx (sensor_id, type);
    float *color = &colors[color_idx];

    checkbox_exist[id] = 1;

    char markup[50];
    PangoColor pc;
    pc.red   = color[0]*65535;
    pc.green = color[1]*65535;
    pc.blue  = color[2]*65535;

    // trick to get colored box
    gchar* pango_color = pango_color_to_string(&pc);
    snprintf(markup, sizeof(markup), "<span background=\"%s\">     </span> ", pango_color);
    GtkBox *hb = GTK_BOX (gtk_hbox_new (FALSE, 0));
    GtkWidget *label = gtk_label_new(NULL);
    gtk_label_set_markup (GTK_LABEL (label), markup);
    gtk_box_pack_start (GTK_BOX (hb), label, FALSE, FALSE, 0);
    GtkWidget *cb = gtk_check_button_new_with_label(name);
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (cb), TRUE);

    gtk_box_pack_start (GTK_BOX (hb), cb, FALSE, FALSE, 0);

    // @todo need to cleanup when button is destroyed
    checkbox_info_t* info = new checkbox_info_t;
    info->renu = this;
    info->id = id;

    // added by mfallon: TODO: check if this vector needs to be cleared when doing "clear all"
    cb_ptr.push_back(cb); 

    g_signal_connect (G_OBJECT(cb), "toggled", G_CALLBACK (on_checkbox_changed), info);
    gtk_widget_show_all (GTK_WIDGET (hb));
    gtk_box_pack_start (GTK_BOX(cbbox), GTK_WIDGET(hb), TRUE, TRUE, 0);

    checkbox_on[id] = 1;
}

LinepltProp::LinepltProp (BotParam *_param, char *rootkey)
{
    m_param = _param;

    char key[256] = {'\0'};

    // nodes
    snprintf (key, sizeof key, "viewer.renderers.%s.pltprop.node", rootkey);
    char *node_lprop = NULL; bot_param_get_str (m_param, key, &node_lprop);
    if (node_lprop) 
        set_line_prop (node_lprop, 0, TYPE_NODE);
    else {
        printf ("No line prop. for nodes provided. Use default: l.\n");
        set_line_prop ((char*)"l.", 0, TYPE_NODE);
    }

    // odometry
    snprintf (key, sizeof key, "viewer.renderers.%s.pltprop.odo", rootkey);
    char *odo_lprop = NULL; bot_param_get_str (m_param, key, &odo_lprop);
    if (odo_lprop) 
        set_line_prop (odo_lprop, perllcm::isam_vlink_t::SENSOR_ODOMETRY, TYPE_VLINK);
    else {
        printf ("No line prop. for odometry provided. Use default: b-\n");
        set_line_prop ((char*)"b-", perllcm::isam_vlink_t::SENSOR_ODOMETRY, TYPE_VLINK);
    }

    // camera line properties
    snprintf (key, sizeof key, "viewer.renderers.%s.pltprop.cam", rootkey);
    int n_cam_lprop = bot_param_get_array_len (m_param, key);
    if (n_cam_lprop == 2) {
        char **cam_lprops = bot_param_get_str_array_alloc (m_param, key);
        set_line_prop (cam_lprops[0], perllcm::isam_vlink_t::SENSOR_CAMERA, TYPE_VLINK);
        set_line_prop (cam_lprops[1], perllcm::isam_vlink_t::SENSOR_CAMERA, TYPE_PLINK);
        bot_param_str_array_free (cam_lprops);
    }
    else {
        printf ("No line prop. for camera provided. Use default: r- and r.\n");
        set_line_prop ((char*)"r-", perllcm::isam_vlink_t::SENSOR_CAMERA, TYPE_VLINK);
        set_line_prop ((char*)"r.", perllcm::isam_vlink_t::SENSOR_CAMERA, TYPE_PLINK);
    }
    // laser line properties
    snprintf (key, sizeof key, "viewer.renderers.%s.pltprop.laser", rootkey);
    int n_laser_lprop = bot_param_get_array_len (m_param, key);
    if (n_laser_lprop == 2) {
        char **laser_lprops = bot_param_get_str_array_alloc (m_param, key);
        set_line_prop (laser_lprops[0], perllcm::isam_vlink_t::SENSOR_LASER, TYPE_VLINK);
        set_line_prop (laser_lprops[1], perllcm::isam_vlink_t::SENSOR_LASER, TYPE_PLINK);
        bot_param_str_array_free (laser_lprops);
    }
    else {
        printf ("No line prop. for laser provided. Use default: g- and g.\n");
        set_line_prop ((char*)"g-", perllcm::isam_vlink_t::SENSOR_LASER, TYPE_VLINK);
        set_line_prop ((char*)"g.", perllcm::isam_vlink_t::SENSOR_LASER, TYPE_PLINK);
    }

    // sonar line properties
    snprintf (key, sizeof key, "viewer.renderers.%s.pltprop.sonar", rootkey);
    int n_sonar_lprop = bot_param_get_array_len (m_param, key);
    if (n_sonar_lprop == 2) {
        char **sonar_lprops = bot_param_get_str_array_alloc (m_param, key);
        set_line_prop (sonar_lprops[0], perllcm::isam_vlink_t::SENSOR_SONAR, TYPE_VLINK);
        set_line_prop (sonar_lprops[1], perllcm::isam_vlink_t::SENSOR_SONAR, TYPE_PLINK);
        bot_param_str_array_free (sonar_lprops);
    }
    else {
        printf ("No line prop. for sonar provided. Use default: p- and p.\n");
        set_line_prop ((char*)"p-", perllcm::isam_vlink_t::SENSOR_SONAR, TYPE_VLINK);
        set_line_prop ((char*)"p.", perllcm::isam_vlink_t::SENSOR_SONAR, TYPE_PLINK);
    }
    
    // glc line properties
    snprintf (key, sizeof key, "viewer.renderers.%s.pltprop.glc", rootkey);
    int n_glc_lprop = bot_param_get_array_len (m_param, key);
    if (n_glc_lprop == 2) {
        char **glc_lprops = bot_param_get_str_array_alloc (m_param, key);
        set_line_prop (glc_lprops[0], perllcm::isam_vlink_t::SENSOR_GLC, TYPE_VLINK);
        set_line_prop (glc_lprops[1], perllcm::isam_vlink_t::SENSOR_GLC, TYPE_PLINK);
        bot_param_str_array_free (glc_lprops);
    }
    else {
        printf ("No line prop. for glc provided. Use default: g- and g.\n");
        set_line_prop ((char*)"r-", perllcm::isam_vlink_t::SENSOR_GLC, TYPE_VLINK);
        set_line_prop ((char*)"r.", perllcm::isam_vlink_t::SENSOR_GLC, TYPE_PLINK);
    }
}

// Line property class
//---------------------------------------------
float*
LinepltProp::get_color (int32_t sensor_id, int type)
{

    int color_idx = get_color_idx (sensor_id, type);
    float *color_found = &colors[color_idx];

    return color_found;
}

int
LinepltProp::get_color_idx (int32_t sensor_id, int type)
{
    int combined_id = sensor_id + type;
    int color_idx = (m_color_idx.find(combined_id)==m_color_idx.end())?GRAY:m_color_idx[combined_id];

    return color_idx;
}

void
LinepltProp::set_line_prop (char *lprop, int32_t sensor_id, int type)
{
    int combined_id = sensor_id + type;

    char _color  = lprop[0];
    char _lstyle = lprop[1];

    int color_idx = GRAY;
    switch (_color) {
        case 'r': color_idx = RED; break;
        case 'l': color_idx = LIME; break;
        case 'b': color_idx = BLUE; break;
        case 'g': color_idx = GREEN; break;
        case 'p': color_idx = PINK; break;
        case 'c': color_idx = CYAN; break;
        default:  color_idx = GRAY; break;
    }
    m_color_idx[combined_id] = color_idx;

    switch (_lstyle) {
        case '-': m_line_type[combined_id] = SOLID; break;
        case ':': m_line_type[combined_id] = DOTTED; break;
        case '=': m_line_type[combined_id] = THICK; break;
        case '.': m_line_type[combined_id] = DOT; break;
        case '*': m_line_type[combined_id] = THICK_DOT; break;
    }
}

void
LinepltProp::apply_line_style (int32_t sensor_id, int type)
{
    int combined_id = sensor_id + type;

    // line style
    int line_style = (m_line_type.find(combined_id)==m_line_type.end())?SOLID:m_line_type[combined_id];
    switch (line_style) {
        case SOLID:
            glLineWidth (1);
            break;
        case DOTTED:
            glLineWidth (1);
            glEnable (GL_LINE_STIPPLE);
            glLineStipple (3, 0b0011001100110011);
            break;
        case THICK:
            glLineWidth (2);
            break;
        case DOT:
            glPointSize (2);
            glBegin (GL_POINTS);
            break;
        case THICK_DOT:
            glPointSize (3);
            glBegin (GL_POINTS);
            break;
        default:
            break;
    }

    // color
    float *_color = get_color (sensor_id, type);
    glColor4f(_color[0], _color[1], _color[2], 1.0);

}

