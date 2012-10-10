#ifndef _RENDERER_UTIL_H
#define _RENDERER_UTIL_H

#include <map>

#include <bot_vis/bot_vis.h>
#include <bot_param/param_client.h>

#include "perls-lcmtypes++/perllcm/isam_vlink_t.hpp"

// checkbox id = sensor_id + TYPE
#define TYPE_NODE    100
#define TYPE_PLINK   200
#define TYPE_VLINK   300

typedef enum _color_idx_t
{
    RED   = 3*0,
    LIME  = 3*1,
    BLUE  = 3*2,
    GREEN = 3*4,
    GRAY  = 3*6,
    PINK  = 3*8,
    CYAN  = 3*9,
} color_idx_t;

// configuration level line property
class LinepltProp {
public:
    LinepltProp(BotParam *_param, char *rootkey);
    ~LinepltProp(){};

    float* get_color (int32_t sensor_id, int type);
    int    get_color_idx (int32_t sensor_id, int type);

    void apply_line_style (int32_t sensor_id, int type);
private:
    BotParam *m_param;

    void set_line_prop (char *lprop, int32_t sensor_id, int type);

    std::map<int32_t, int> m_color_idx;
    std::map<int32_t, int> m_line_type;

};

// dynamic checkbox on lcm messages
typedef struct {
    // data=[x,y,z,r,p,h]       for nodes
    // data=[x1,y1,z1,x2,y2,z2] for links
    double   data[6];
    int32_t  sensor_id;
} dyncb_vis_data_t;

typedef struct {
    int32_t m_type;
    std::vector<dyncb_vis_data_t> m_dyncb_vis_data;    // <sensor id, data>
} dyncb_vis_item_t;

class DynCheckbox {
public:
    DynCheckbox(BotViewer *_viewer, GtkWidget* vbox, LinepltProp *_lineprop){
        viewer = _viewer;
        cbMutex = g_mutex_new();
        cbbox = gtk_vbox_new (FALSE, 0);
        gtk_container_add (GTK_CONTAINER (vbox), cbbox);
        gtk_widget_show (cbbox);
        m_lineprop = _lineprop;
    };

    ~DynCheckbox(){
        g_mutex_free (cbMutex);

        // @todo: free cbbox and cb_ptr ??
        //std::vector<GtkWidget*>::iterator it_cb_ptr;
        //for (it_cb_ptr=m_palette.begin(); it_cb_ptr<m_palette.end(); it_cb_ptr++)
        //    delete (it_cb_ptr->second);
    };

    BotViewer *viewer;

    // checkboxes
    GtkWidget *cbbox;
    std::vector<GtkWidget*> cb_ptr;
    GMutex* cbMutex;
    std::map<int32_t, bool> checkbox_exist;
    std::map<int32_t, bool> checkbox_on;

    void add_checkbox (const char* name, int sensor_id, int type);

private:
    LinepltProp *m_lineprop;
};


#endif
