#include "map_builder.h"

using namespace std;


/**
 * @brief PVNMapBuilder Constructor
 */
PVNMapBuilder::PVNMapBuilder (int argc, char *argv[])
{
    // open param cfg file
    param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        std::cout << "could not find " << BOTU_PARAM_DEFAULT_CFG << std::endl;
    }
    std::cout << "Opened Param File [" << BOTU_PARAM_DEFAULT_CFG << "]" << std::endl;
    
    // read from param file
    scan_dir = botu_param_get_str_or_default (param, "velodyne-scan-matcher.logdir", ".");
    graph_dir = botu_param_get_str_or_default (param, "pvn-map-builder.graph_dir", ".");
    las_dir = botu_param_get_str_or_default (param, "pvn-map-builder.las_dir", ".");
    map_dir = botu_param_get_str_or_default (param, "pvn-map-builder.map_dir", ".");
    image_dir = botu_param_get_str_or_default (param, "pvn-map-builder.image_dir", ".");
    map_name = botu_param_get_str_or_default (param, "pvn-map-builder.map_name", "pvn_map");
    memset (use_camera, 1, 6*sizeof (int));
    bot_param_get_int_array (param, "pvn-map-builder.use_camera", use_camera, 6);
    
    las_subsample_pct = 0.1;
    bot_param_get_double (param, "pvn-map-builder.las_subsample_pct", &las_subsample_pct);
    
    lm_force_lite = 0;
    bot_param_get_int (param, "pvn-map-builder.lm_force_lite", &lm_force_lite);
    lm_every_nth_return = 0;
    bot_param_get_int (param, "pvn-map-builder.lm_every_nth_return", &lm_every_nth_return);
    
    // gps linearization
    double ll_deg[2] = {0};
    bot_param_get_double_array_or_fail (param, "site.orglatlon", ll_deg, 2);
    llxy = (BotGPSLinearize *)calloc(1, sizeof (*llxy));
    bot_gps_linearize_init (llxy, ll_deg);
    org_alt = 0;
    bot_param_get_double (param, "site.orgalt", &org_alt);
    
    // read command line args
    gopt = getopt_create ();
    
    char las_sub_pct_str[128] = {0};
    sprintf (las_sub_pct_str, "%0.4lf", las_subsample_pct);
    char lm_force_lite_str[128] = {0};
    sprintf (lm_force_lite_str, "%d", lm_force_lite);
    char lm_every_nth_str[128] = {0};
    sprintf (lm_every_nth_str, "%d", lm_every_nth_return);
    getopt_add_description (gopt, "Persistent Visual Navigation Map Builder");
    getopt_add_bool   (gopt,  '\0', "build_seed_map",   0,                  "Build the seed map from Navteq seed data");
    getopt_add_bool   (gopt,  '\0', "build_laser_map",  0,                  "Build the laser map");
    getopt_add_bool   (gopt,  '\0', "build_eview_map",  0,                  "Build the eview map");
    getopt_add_bool   (gopt,  '\0', "build_vocab",      0,                  "Build visual vocabulary");
    getopt_add_bool   (gopt,  '\0', "build_cltree",     0,                  "Build Chow-Liu tree");
    getopt_add_bool   (gopt,  '\0', "sparsify",         0,                  "Sparisfy Map");
    getopt_add_string (gopt,  '\0', "eview_process",    "",                 "Process eview map data");
    getopt_add_spacer (gopt, "--------------- shared opts --------------------");
    getopt_add_string (gopt,  's',  "scan_dir",         scan_dir,           "Directory with velodyne scans (seed, laser and vis)");
    getopt_add_string (gopt,  'm',  "map_dir",          map_dir,            "Directory with maps (seed, laser, and vis)");
    getopt_add_string (gopt,  'g',  "graph_dir",        graph_dir,          "Path to graph pose collection (laser and vis)");
    getopt_add_string (gopt,  'n',  "map_name",         map_name,           "Map name prefix (seed, laser, and vis)");
    getopt_add_spacer (gopt, "--------------- seed map opts ------------------");
    getopt_add_string (gopt,  's',  "las_dir",          las_dir,            "Directory with Navteq .las scan files");
    getopt_add_double (gopt,  '\0', "las_subs_pct",     las_sub_pct_str,    "Percentage to subsample Navteq .las scan files");
    getopt_add_spacer (gopt, "--------------- laser map opts -----------------");
    getopt_add_int    (gopt,  '\0', "lm_force_lite",    lm_force_lite_str,  "Force returns to lite def");
    getopt_add_int    (gopt,  '\0', "lm_every_nth",     lm_every_nth_str,   "Skip every nth return");
    getopt_add_bool   (gopt,  '\0', "lm_color",         0,                  "Compute color information");
    getopt_add_spacer (gopt, "----------------eview map opts -----------------");
    getopt_add_int    (gopt,  '\0', "eview_backup",     "0",                "Restart from i = eview_backup");
    getopt_add_string (gopt,  '\0', "eview_end_at",     "0",                "End eview map at this timestamp");
    getopt_add_spacer (gopt, "------------------------------------------------");
    getopt_add_bool   (gopt,  'h',  "help",    	        0,                  "Display Help");

    if (!getopt_parse (gopt, argc, argv, 1)) {
        getopt_do_usage (gopt,"");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt,"");
        exit (EXIT_SUCCESS);
    }
    
    strcpy (scan_dir, getopt_get_string (gopt, "scan_dir"));
    strcpy (las_dir, getopt_get_string (gopt, "las_dir"));
    strcpy (map_dir, getopt_get_string (gopt, "map_dir"));
    strcpy (graph_dir, getopt_get_string (gopt, "graph_dir"));
    strcpy (map_name, getopt_get_string (gopt, "map_name"));
    las_subsample_pct = getopt_get_double (gopt, "las_subs_pct");
    lm_force_lite = getopt_get_int (gopt, "lm_force_lite");
    lm_every_nth_return = getopt_get_int (gopt, "lm_every_nth");
    lm_color = getopt_get_bool (gopt, "lm_color");
    eview_backup = getopt_get_int (gopt, "eview_backup");
    if(getopt_has_flag (gopt, "eview_end_at"))
        sscanf (getopt_get_string (gopt, "eview_end_at"), "%ld", &eview_end_at);
    else
        eview_end_at = 0;
    
    // check lcm connection
    if(!lcm.good()) {
        is_done = 1;
        ERROR ("lcm_create () failed!");
    }
}

PVNMapBuilder::~PVNMapBuilder ()
{
    bot_param_destroy (param);
    free (llxy);
}

// ----------------------------------------------------------------------------
// Main Loop
// ----------------------------------------------------------------------------
void
PVNMapBuilder::run ()
{    
    
    // check with task
    if (!getopt_get_bool (gopt, "build_laser_map") &&
        !getopt_get_bool (gopt, "build_seed_map") &&
        !getopt_get_bool (gopt, "build_eview_map") &&
        !getopt_get_bool (gopt, "build_vocab") &&
        !getopt_get_bool (gopt, "build_cltree") &&
        !getopt_get_bool (gopt, "sparsify") &&
        !getopt_has_flag (gopt, "eview_process") &&
        !getopt_get_bool (gopt, "cluster_vis_map")) {
        cout << "No job selected ... exiting." << endl;
        exit (EXIT_SUCCESS);
    }
    
    if (getopt_get_bool (gopt, "build_vocab"))
        run_build_vocab ();
        
    if (getopt_get_bool (gopt, "build_cltree"))
        run_build_chow_liu_tree();
    
    if (getopt_get_bool (gopt, "build_seed_map"))
        run_build_seed_map ();
    
    if (getopt_get_bool (gopt, "build_laser_map"))
        run_build_laser_map ();
        
    if (getopt_get_bool (gopt, "build_eview_map"))
        run_build_eview_map ();
        
    if (getopt_get_bool (gopt, "sparsify"))
        run_sparsify_map ();
        
    if (getopt_has_flag (gopt, "eview_process")) {
        char tmp[PATH_MAX];
        strcpy (tmp, getopt_get_string (gopt, "eview_process"));
        run_eview_process (tmp);
    }
}
        
// ----------------------------------------------------------------------------
// Ctrl+C catching
// ----------------------------------------------------------------------------
PVNMapBuilder *g_pvnmb;

void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    std::cout << "Sigint caught. Quitting ..." << std::endl;
    if (g_pvnmb->is_done ) {
        std::cout << "Goodbye" << std::endl;
        exit (EXIT_FAILURE);
    } 
    else
        g_pvnmb->is_done = 1;
}

// ----------------------------------------------------------------------------
// Main 
// ----------------------------------------------------------------------------
int
main (int argc, char *argv[])
{    
    fasttrig_init ();
    
    PVNMapBuilder pvnmb = PVNMapBuilder::PVNMapBuilder (argc, argv);
    g_pvnmb = &pvnmb;
    
    // install custom signal handler
    struct sigaction act;
    act.sa_sigaction = my_signal_handler;
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);
    
    // kick off the manager
    pvnmb.run ();
    
    return 0;    
}
