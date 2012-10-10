#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <assert.h>

// external linking req'd
#include <bot_core/bot_core.h>
#include <bot_lcmgl_render/lcmgl_bot_renderer.h>
#include <bot_vis/bot_vis.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <gtk/gtk.h>

#include "perls-math/fasttrig.h"

#include "perls-common/bot_util.h"
#include "perls-common/udp.h"

#include "renderers.h"

#define LOGPLAYER_UDP_PORT 53261


typedef struct _state_t state_t;
struct _state_t {    
    BotViewer *viewer;
    BotEventHandler *ehandler;
    
    lcm_t *lcm;
    
    int logplayer_udp_port;

    BotParam *param;
    
    char view_handler_owner_id_key[256];
};

state_t state = {0};


int
is_view_handler_owner (char *id_key)
{    
    if (0 == strcmp (state.view_handler_owner_id_key, id_key) ||
        0 == strcmp (state.view_handler_owner_id_key, ""))
        return 1;
    else
        return 0;
}

void
own_view_handler (char *id_key)
{
    strcpy (state.view_handler_owner_id_key, id_key); 
}


static int 
logplayer_remote_on_key_press (BotViewer *viewer, BotEventHandler *ehandler, 
                               const GdkEventKey *event)
{
    int keyval = event->keyval;

    if (!state.logplayer_udp_port)
        return 0;

    switch (keyval) {
    case 'P':
    case 'p':
        // go into pause mode.
        udp_send_string ("127.0.0.1", state.logplayer_udp_port, "PLAYPAUSETOGGLE");
        break;
    case 'N':
    case 'n':
        udp_send_string ("127.0.0.1", state.logplayer_udp_port, "STEP");
        break;
    case '=':
    case '+':
        udp_send_string ("127.0.0.1", state.logplayer_udp_port, "FASTER");
        break;
    case '_':
    case '-':
        udp_send_string ("127.0.0.1", state.logplayer_udp_port, "SLOWER");
        break;
    case '[':
        udp_send_string ("127.0.0.1", state.logplayer_udp_port, "BACK5");
        break;
    case ']':
        udp_send_string ("127.0.0.1", state.logplayer_udp_port, "FORWARD5");
        break;
    default:
        return 0;
    }

    return 1;
}


int
main (int argc, char *argv[])
{
    gtk_init (&argc, &argv);
    glutInit (&argc, argv);
    
    fasttrig_init ();

    if (!g_thread_supported ()) 
        g_thread_init (NULL);

    setlinebuf (stdout);

    state.viewer = bot_viewer_new ("PeRL Viewer");
    
    // set pale non white background color
    state.viewer->backgroundColor[0] = 0.95;
    state.viewer->backgroundColor[1] = 0.95;
    state.viewer->backgroundColor[2] = 1;
    state.viewer->backgroundColor[3] = 1;
    
    state.lcm = bot_lcm_get_global (NULL);
    state.logplayer_udp_port = LOGPLAYER_UDP_PORT;
    
    //important, otherwise lcm doesnt work right in renderers
    bot_glib_mainloop_attach_lcm (state.lcm);
       
    // logplayer controls
    state.ehandler = (BotEventHandler*) calloc (1, sizeof (BotEventHandler));
    state.ehandler->name = (char*) "LogPlayer Remote";
    state.ehandler->enabled = 1;
    state.ehandler->key_press = logplayer_remote_on_key_press;
    bot_viewer_add_event_handler (state.viewer, state.ehandler, 1);
    
    bot_viewer_add_stock_renderer (state.viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1000);

    // setup renderers from config file
    state.param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    int num_rends = bot_param_get_num_subkeys (state.param, "viewer.renderers");
    char **rend_keys = bot_param_get_subkeys (state.param, "viewer.renderers");
    printf ("Number of Renders %d \n", num_rends);

    // loop over each observation model
    for (int i=0; i<num_rends; i++) {
        char key[256] = {0};

        snprintf (key, sizeof key, "viewer.renderers.%s.renderer_key", rend_keys[i]);
        char *renderer_type = bot_param_get_str_or_fail (state.param, key);
        
        printf ("Adding Renderer: %s \n", rend_keys[i]);
    
        if (0==strcmp ("robot_pose", renderer_type)) {
            
            robot_pose_data_t *pose_data = setup_renderer_robot_pose (state.viewer, rend_keys[i], 0);

            // setup any renderers that attach to the pose of this robot
            snprintf (key, sizeof key, "viewer.renderers.%s.pose_attachments", rend_keys[i]);
            int num_attachments = bot_param_get_num_subkeys (state.param, key);
            printf ("\tNumber of Robot Pose attachments %d \n", num_attachments);
            char **pose_attachment_keys;
            pose_attachment_keys = bot_param_get_subkeys (state.param, key);

            // loop over each pose attachment
            for (int j=0; j<num_attachments; j++) {
                
                printf ("\tAdding Pose Attachment Renderer: %s \n", pose_attachment_keys[j]);
                
                snprintf (key, sizeof key, "viewer.renderers.%s.pose_attachments.%s.renderer_key", 
                          rend_keys[i], pose_attachment_keys[j]);
                char *attach_type = bot_param_get_str_or_fail (state.param, key);
                
                char cfg_path[512];
                snprintf (cfg_path, sizeof cfg_path, "viewer.renderers.%s.pose_attachments.%s",
                          rend_keys[i], pose_attachment_keys[j]);
                
                if (0==strcmp ("dvl_beams", attach_type))
                    setup_renderer_dvl_beams (state.viewer, cfg_path, pose_data, 0);
                else if (0==strcmp ("range_circles", attach_type))
                    setup_renderer_range_circles (state.viewer, cfg_path, pose_data, 0);
                else if (0==strcmp ("velodyne", attach_type))
                    setup_renderer_velodyne (state.viewer, cfg_path, pose_data, 0);
                else if (0==strcmp ("planar_laser", attach_type))
                    setup_renderer_planar_laser (state.viewer, cfg_path, pose_data, 0);
                else if (0==strcmp ("planar_target", attach_type))
                    setup_renderer_planar_target (state.viewer, cfg_path, pose_data, 0);
                else {
                    printf ("\tATTACHED RENDERER NOT FOUND: %s \n", attach_type);
                    abort ();
                }
            }
        }
        else if (0==strcmp ("image", renderer_type))
            setup_renderer_image (state.viewer, rend_keys[i], 0);
        else if (0==strcmp ("status_text", renderer_type))
            setup_renderer_status_text (state.viewer, rend_keys[i], 0);
        else if (0==strcmp ("isam_graph", renderer_type))
            setup_renderer_isam_graph (state.viewer, rend_keys[i], 0);
        else if (0==strcmp ("pvn_map", renderer_type))
            setup_renderer_pvn_map (state.viewer, rend_keys[i], 0);
        else if (0==strcmp ("sat_map", renderer_type))
            setup_renderer_sat_map (state.viewer, rend_keys[i], 999);
        else if (0==strcmp ("compass", renderer_type))
            setup_renderer_compass (state.viewer, 0);
        else if (0==strcmp ("bot_lcmgl", renderer_type))
            bot_lcmgl_add_renderer_to_viewer(state.viewer,state.lcm, 0);
        else if (0==strcmp ("lcmgl", renderer_type))
            setup_renderer_lcmgl (state.viewer, rend_keys[i], 0);       //@todo: why not use bot_lcmgl?
        else if (0==strcmp ("hauv", renderer_type))
            setup_renderer_hauv (state.viewer, rend_keys[i], 0);
        else if (0==strcmp ("vanctrl", renderer_type))
            setup_renderer_vanctrl (state.viewer, rend_keys[i], 0);
        else if (0==strcmp ("hauv_pose_attach", renderer_type))
            setup_renderer_hauv_pose (state.viewer, rend_keys[i], 0);
        else if (0==strcmp ("camctrl", renderer_type))
            setup_renderer_camctrl (state.viewer, rend_keys[i], 0);
        else if (0==strcmp("plot", renderer_type))
            setup_renderer_plot (state.viewer, rend_keys[i], 0);
        else if (0==strcmp ("vectormap_mission", renderer_type))
            setup_renderer_vectormap_mission (state.viewer, rend_keys[i], 0);              
        else {
            printf ("RENDERER NOT FOUND: %s \n", renderer_type);
            abort ();
        }
    }

    char *fname = g_build_filename (g_get_user_config_dir (), ".perl-viewer", NULL);
    bot_viewer_load_preferences (state.viewer, fname);
    
    gtk_main ();
    
    bot_viewer_save_preferences (state.viewer, fname);
    bot_viewer_unref (state.viewer);
    free (fname);
    
    exit (EXIT_SUCCESS);
}
