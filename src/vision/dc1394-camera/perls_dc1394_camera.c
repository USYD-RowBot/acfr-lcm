#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <sys/select.h>
#include <dc1394/dc1394.h>

#include "perls_dc1394_camera.h"

#define FORMAT7_BUS_MULTIPLIER 1.05

// Parse timestamp
uint32_t
cycle_timer_to_usec (uint32_t cycle)
{
    return (((uint32_t)cycle >> 25) & 0x7f) * 1000000 + 
           (((uint32_t)cycle & 0x01fff000) >> 12) * 125 +
           ((uint32_t)cycle & 0x00000fff) * 125 / 3072;   
}

dc1394error_t
perls_dc1394_camera_parse_embedded_timestamp (PERLS_dc1394_camera_t *perls_cam, uint8_t *image, uint64_t *timestamp)
{   
    dc1394error_t status;
    uint64_t bus_timestamp;
    uint32_t cycle_usec_now;
    uint64_t systime = 0;
    uint32_t cyctime;

    // get embedded timestamp from first 4 bytes of image data
    bus_timestamp = (image[0] << 24) | (image[1] << 16) | (image[2] << 8) | image[3];

    // bottom 4 bits of cycle offset will be a frame count
    bus_timestamp &= 0xfffffff0;

    status = dc1394_read_cycle_timer (perls_cam->cam, &cyctime, &systime);
    if (status != DC1394_SUCCESS) {
        *timestamp = cycle_timer_to_usec (bus_timestamp);
        return status;
    }

    cycle_usec_now = cycle_timer_to_usec (cyctime);

    int usec_diff = cycle_usec_now - cycle_timer_to_usec (bus_timestamp);
    if (usec_diff < 0)
        usec_diff += CYCLE_TIMER_MAX_USEC;

    *timestamp = systime - usec_diff;

    return status;
}

/**
 * Lists all cameras on the bus by camera_id
 */
dc1394error_t
perls_dc1394_camera_list (void)
{
    dc1394_t *dc1394 = dc1394_new ();
    if (!dc1394) {
        printf ("Returning dc1394 is null\n");
        return DC1394_FAILURE;
    }

    dc1394camera_list_t *camlist;
    if (dc1394_camera_enumerate (dc1394, &camlist) < 0) {
        dc1394_free (dc1394);
        printf ("No camera found\n");
        return DC1394_FAILURE;
    }

    for (int i=0; i < camlist->num; ++i) {
        dc1394camera_t *cam = dc1394_camera_new (dc1394, camlist->ids[i].guid);
        printf ("%s %s: ID=%lx\n", cam->vendor, cam->model, cam->guid);
        dc1394_camera_free (cam);
        cam = NULL;
    }

    dc1394_camera_free_list (camlist);
    dc1394_free (dc1394);

    return DC1394_SUCCESS;
}

/**
 * Queries the cam_id for supported video modes/framerates.
 */
dc1394error_t
perls_dc1394_camera_query (const char *cam_id)
{
    dc1394_t *dc1394 = dc1394_new ();
    if (!dc1394) {
        printf ("Returning dc1394 is null\n");
        return DC1394_FAILURE;
    }

    uint64_t id;
    sscanf (cam_id, "%lx", &id);
    dc1394camera_t *cam = dc1394_camera_new (dc1394, id);
    if (!cam) {
        printf ("Error: Specified camera not found.\n");
        dc1394_free (dc1394);
        return DC1394_FAILURE;
    }

    printf ("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf ("%s %s: %lx\n", cam->vendor, cam->model, cam->guid);
    printf ("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

    //video mode, color, framerates
    dc1394video_modes_t video_modes;
    dc1394framerates_t framerates;
    dc1394_video_get_supported_modes (cam, &video_modes);

    printf ("Standard Video Modes:\n");
    printf ("These modes can be set to any framerate not exceeding the maximum listed framerate.\n");
    for (int x = 0; x < video_modes.num; ++x) {

        if (!dc1394_is_video_mode_scalable (video_modes.modes[x])) {
            char *tmp = dc1394_videomode_int_to_string (video_modes.modes[x]);
            printf ("\t%s\n", tmp); free (tmp);
            printf ("\t\t-Standard Framerates = ");
            dc1394_video_get_supported_framerates (cam, video_modes.modes[x], &framerates);
            for (int f = 0; f < framerates.num; ++f) {
                printf ("%g", dc1394_framerate_int_to_double (framerates.framerates[f]));
                if (f!=framerates.num-1) printf (", ");
                else printf ("\n");
            }
        }
    }

    uint32_t h, v;
    uint32_t h_unit, v_unit;
    uint32_t h_unit_pos, v_unit_pos;
    uint32_t unit_bytes, max_bytes;
    double fps;
    dc1394color_codings_t codings;

    printf ("Scalable Video Modes (FORMAT7):\n");
    printf ("These modes cannot directly control framerate, the user must select a target framerate below max.\n");
    printf ("Note: Assumed ISO mode for fps estimates is %s\n", cam->bmode_capable>0?"800":"400");
    for (int x = 0; x < video_modes.num; ++x) {

        if (dc1394_is_video_mode_scalable (video_modes.modes[x])) {
            char *tmp = dc1394_videomode_int_to_string (video_modes.modes[x]);
            printf ("\t%s\n", tmp); free (tmp);
            dc1394_format7_get_max_image_size (cam, video_modes.modes[x], &h, &v);
            printf ("\t\t-Max image width = %d, height = %d\n", h, v);
            dc1394_format7_get_unit_size (cam, video_modes.modes[x], &h_unit, &v_unit);
            printf ("\t\t-Units for image width = %d, height = %d\n", h_unit, v_unit);
            dc1394_format7_get_unit_position (cam, video_modes.modes[x], &h_unit_pos, &v_unit_pos);
            printf ("\t\t-Units for image startX = %d, startY = %d\n", h_unit_pos, v_unit_pos);
            dc1394_format7_get_packet_parameters (cam, video_modes.modes[x], &unit_bytes, &max_bytes);

            dc1394_format7_set_roi (cam, video_modes.modes[x], DC1394_COLOR_CODING_MONO8, DC1394_USE_MAX_AVAIL, 0, 0, h, v);
            fps = max_bytes * (cam->bmode_capable > 0 ? 8000.0 : 4000.0) / h / v / FORMAT7_BUS_MULTIPLIER;
            printf ("\t\t-Max framerate @ %dx%d and 1 byte/pixel = %4.1f fps\n", h, v, fps);
            printf ("\t\t-Packet max bytes = %d, unit bytes = %d\n", max_bytes, unit_bytes);
            dc1394_format7_get_color_codings (cam, video_modes.modes[x], &codings);
            for (int i=0; i < codings.num; ++i) {
                tmp = dc1394_colorcode_int_to_string (codings.codings[i]);

                if (i == 0) printf ("\t\tColor codings: ");
                printf ("%s", tmp); free (tmp);
                if (i != codings.num-1) printf (", ");
                else printf ("\n");
            }
        }
    }

    dc1394_camera_free (cam);
    dc1394_free (dc1394);

    return DC1394_SUCCESS;
}


/**
 * Cleans and deallocates all bandwidth for all cameras
 */
dc1394error_t
perls_dc1394_camera_clean (void)
{
    dc1394_t *dc1394 = dc1394_new ();
    if (!dc1394) {
        printf ("Returning dc1394 is null\n");
        return DC1394_FAILURE;
    }

    dc1394camera_list_t *camlist;
    if (dc1394_camera_enumerate (dc1394, &camlist) < 0) {
        dc1394_free (dc1394);
        printf ("No camera found\n");
        return DC1394_FAILURE;
    }
    dc1394camera_list_t *active_camlist;

    for (int i=0; i < camlist->num; ++i) {
        dc1394_camera_enumerate (dc1394, &active_camlist);
        for (int x=0; x< active_camlist->num; ++x) {
            if (camlist->ids[i].guid == active_camlist->ids[x].guid) {
                dc1394camera_t *cam = dc1394_camera_new (dc1394, active_camlist->ids[x].guid);
                if (cam == NULL || dc1394_reset_bus (cam) != DC1394_SUCCESS)
                    return DC1394_FAILURE;
                dc1394_camera_free (cam);
                break;
            }
        }
        dc1394_camera_free_list (active_camlist);
    }

    dc1394_camera_free_list (camlist);
    dc1394_free (dc1394);

    return DC1394_SUCCESS;
}
        

/**
 * Looks for the cam_id camera connected
 * to the computer and initializes it. It returns a pointer 
 * to the camera handle on success and returns "NULL" on 
 * failure.
 */
PERLS_dc1394_camera_t * 
perls_dc1394_camera_init (const char * cam_id)
{
    PERLS_dc1394_camera_t *perls_cam = malloc (sizeof (*perls_cam));
    dc1394camera_t *cam = NULL;
    dc1394_t *dc1394 = dc1394_new ();
    if (!dc1394) {
        printf ("Returning dc1394 is null\n"); 
        return NULL;
    }

    dc1394camera_list_t *camlist;
    if (dc1394_camera_enumerate (dc1394, &camlist) < 0) {
        dc1394_free (dc1394);
        printf ("No camera found\n");
        return NULL;
    }
  
    //loop over all firewire cameras available
    for (int i = 0; i < camlist->num; i++) {
        cam = dc1394_camera_new (dc1394, camlist->ids[i].guid);

        if (cam == NULL)
            continue;
     
        char unique_id[17] = {0};
        snprintf (unique_id,17,"%"PRIx64"", cam->guid);        
        
        if (strcmp ("", cam_id) && strcmp (unique_id, cam_id)) { //finds specified ID
            dc1394_camera_free (cam);
            cam = NULL;
            continue;
        }
        
        break;
    }
  
    dc1394_camera_free_list (camlist);
    if (!cam) {
        printf ("Failed to find camera w/ ID = %s\n",cam_id);
        dc1394_free (dc1394);
        return NULL;
    }

    // show some information about the camera we're using
    dc1394_camera_print_info (cam, stdout);
    
    perls_cam->cam = cam;
    perls_cam->dc1394 = dc1394; 

    if (perls_dc1394_camera_cleanup_iso (perls_cam->cam) != DC1394_SUCCESS) {
        printf ("Failed in pre-cleaning bus, perhaps reset bus.\n");
        dc1394_camera_free (cam);
        dc1394_free (dc1394);
        return NULL;
    }

    return perls_cam;
}

/**
 * Clears previously allocated isochronous bandwidth and channels.
 *     -this is essential if a dc1394 camera is not properly closed
 */
dc1394error_t
perls_dc1394_camera_cleanup_iso (dc1394camera_t *cam)
{
    dc1394error_t status;
    uint32_t value;

    // check and clear iso bandwidth
    status = dc1394_video_get_bandwidth_usage (cam, &value);
    DC1394_WRN (status, "getting iso bandwidth");

    if (status == DC1394_SUCCESS) {
        status = dc1394_iso_release_bandwidth (cam, value);
        DC1394_WRN (status, "releasing iso bandwidth");
    }

    // check and clear iso channel
    status = dc1394_video_get_iso_channel (cam, &value);
    DC1394_WRN (status, "getting iso channel");

    if (status == DC1394_SUCCESS) {
        status = dc1394_iso_release_channel (cam, value);
        DC1394_WRN (status, "releasing iso channel");
    }

    return DC1394_SUCCESS;
}

/**
 * Sets the operation mode to 1394A (legacy) or 1394B
 */
dc1394error_t 
perls_dc1394_camera_set_operation_mode (PERLS_dc1394_camera_t *perls_cam, dc1394operation_mode_t mode)
{
    dc1394error_t status;
    status = dc1394_video_set_operation_mode (perls_cam->cam, mode);
    DC1394_ERR_RTN (status, "setting operation mode");
    return status;
}

/**
 * Sets the isochronous speed
 */
dc1394error_t
perls_dc1394_camera_set_iso_speed(PERLS_dc1394_camera_t *perls_cam, dc1394speed_t speed)
{
    //For speed greater than 400 Mbps you need to set the operation mode to 1394B
    dc1394error_t status;
    status = dc1394_video_set_iso_speed (perls_cam->cam, speed);
    DC1394_ERR_RTN (status, "setting speed");
    return status;
}

/**
 * Sets the DC1394 video mode.
 */
dc1394error_t
perls_dc1394_camera_set_video_mode (PERLS_dc1394_camera_t *perls_cam, dc1394video_mode_t vidMode)
{
    dc1394error_t status;
    char *tmp;

    // make sure this camera allows this video mode
    dc1394video_modes_t video_modes;
    status = dc1394_video_get_supported_modes (perls_cam->cam, &video_modes);
    DC1394_ERR_RTN (status, "checking supported video modes");

    if (video_modes.num == 0) {
        ERROR ("No supported video modes found");
        return DC1394_FAILURE;
    }

    if (vidMode == 0) { // auto assign
        int found1280 = -1, found640 = -1;
        for (int i=0; i < video_modes.num; ++i) {
            if (video_modes.modes[i] == DC1394_VIDEO_MODE_640x480_MONO8) found640 = i;
            if (video_modes.modes[i] == DC1394_VIDEO_MODE_1280x960_MONO8) found1280 = i;
        }

        vidMode = (found1280 > -1) ? video_modes.modes[found1280] : 
                    (found640 > -1) ? video_modes.modes[found640] : video_modes.modes[0];

        tmp = dc1394_videomode_int_to_string (vidMode);
        ERROR ("Video mode not specified, setting to [%s]", tmp);
        free (tmp);
    }
    else {
        for (int i=0; i < video_modes.num; ++i) {
            if (vidMode == video_modes.modes[i])
                break;
            if (i == video_modes.num-1) {
                tmp = dc1394_videomode_int_to_string (vidMode);
                ERROR ("Video mode [%s] not supported by camera", tmp);
                free (tmp);
                return DC1394_FAILURE;
            }
        }
    }
    
    status = dc1394_video_set_mode (perls_cam->cam, vidMode);
    DC1394_ERR_RTN (status, "setting video mode");
  
    return status;
}

/**
 * Checks the currently specified DC1394 color code.
 */
dc1394error_t
perls_dc1394_camera_get_color_coding (PERLS_dc1394_camera_t *perls_cam, dc1394color_coding_t *color)
{
    dc1394error_t status;

    dc1394video_mode_t mode;
    status = dc1394_video_get_mode (perls_cam->cam, &mode);
    DC1394_ERR_RTN (status, "checking video mode in get color code");


    if (dc1394_is_video_mode_scalable (mode))
        status = dc1394_format7_get_color_coding (perls_cam->cam, mode, color);
    else
        status = dc1394_get_color_coding_from_video_mode  (perls_cam->cam, mode, color);

    DC1394_ERR_RTN (status, "checking color code");

    return status;
}

/**
 * Checks the Bayer pattern currently in use
 */
dc1394error_t
perls_dc1394_camera_get_bayer_pattern (PERLS_dc1394_camera_t *perls_cam, dc1394color_filter_t *filter)
{
    uint32_t value = 0;
    dc1394error_t status = dc1394_get_control_register (perls_cam->cam,
            PERLS_DC1394_CAMERA_REGISTER_BAYER_TILE_MAPPING, &value);
    DC1394_ERR_RTN (status, "Reading bayer tile mapping register");

    // R=52, G=47, B=42, Y=59
    switch (value) {
        case 0x59595959:    // YYYY
            *filter = (dc1394color_filter_t) 0;
            break;
        case 0x52474742:    // RGGB
            *filter = DC1394_COLOR_FILTER_RGGB;
            break;
        case 0x47425247:    // GBRG
            *filter = DC1394_COLOR_FILTER_GBRG;
            break;
        case 0x47524247:    // GRBG
            *filter = DC1394_COLOR_FILTER_GRBG;
            break;
        case 0x42474752:    // BGGR
            *filter = DC1394_COLOR_FILTER_BGGR;
            break;
        default:
            *filter = (dc1394color_filter_t) 0;
            break;
    }

    return status;
}

/**
 * Sets the framerate of the camera. 
 * If set frame rate is more than the transfer rate then the framerate is governed by the 
 * transfer rate. Note: only use this function on non-scalable video modes.
 */
dc1394error_t
perls_dc1394_camera_set_frame_rate (PERLS_dc1394_camera_t *perls_cam, float framerate)
{

    dc1394error_t status;

    // make sure that this is a non-scalable type
    dc1394video_mode_t mode;
    status = dc1394_video_get_mode (perls_cam->cam, &mode);
    DC1394_ERR_RTN (status, "Must set video mode before setting framerate");
    if (dc1394_is_video_mode_scalable (mode)) {
        ERROR ("Can't set framerate of scalable video modes");
        return DC1394_FAILURE;
    }

    // set maximum frame rate
    dc1394framerates_t framerates;
    status = dc1394_video_get_supported_framerates (perls_cam->cam, mode, &framerates);
    DC1394_ERR_RTN (status, "Can't retrieve supported framerates. Is the video mode not supported?");

    // set limits on framerate
    if (framerate < 0 || framerate > dc1394_framerate_int_to_double (framerates.framerates[framerates.num-1])) {
        framerate = dc1394_framerate_int_to_double (framerates.framerates[framerates.num-1]);
        printf ("Set maximum framerate: %g fps\n", framerate);
    }
    
    // set bus framerate to just above actual framerate
    for (int i=0; i<framerates.num; ++i) {
        if (framerate <= dc1394_framerate_int_to_double (framerates.framerates[i])) {
            status = dc1394_video_set_framerate (perls_cam->cam, framerates.framerates[i]);
            DC1394_ERR_RTN (status, "Can't set default framerate.");
            break;
        }
    }

    // Set frame rate control register to check absolute value frame rate field
    uint32_t value=0;
    value = 0xC2000000; 
    status = dc1394_set_control_register (perls_cam->cam, PERLS_DC1394_CAMERA_REGISTER_FRAME_RATE_CONTROL, value);
    DC1394_ERR_RTN (status, "Cannot set the frame rate register value");

    // set absolute value field of frame rate by copying the IEEE*4 32-bit float to uint
    uint32_t ui;
    memcpy (&ui, &framerate, sizeof (ui));
    status = dc1394_set_control_register (perls_cam->cam, PERLS_DC1394_CAMERA_REGISTER_ABS_FRAME_RATE_CONTROL, ui);
    DC1394_ERR_RTN (status, "Cannot set the frame rate register value");
    return status;
}

/**
 * Sets the ROI for scalable video modes (FORMAT7)
 */
dc1394error_t
perls_dc1394_camera_set_roi (PERLS_dc1394_camera_t *perls_cam, dc1394color_coding_t color,
        double framerate, int left, int top, int width, int height)
{
    dc1394error_t status;

    // make sure that video mode has already been set
    dc1394video_mode_t mode;
    status = dc1394_video_get_mode (perls_cam->cam, &mode);
    DC1394_ERR_RTN (status, "Must set video mode before setting ROI");

    // check that this is a scalable video mode
    if (!dc1394_is_video_mode_scalable (mode)) {
        ERROR ("Can't set ROI because video mode not scalable");
        return DC1394_FAILURE;
    }

    dc1394color_codings_t codings;
    dc1394_format7_get_color_codings (perls_cam->cam, mode, &codings);

    if (codings.num == 0) {
        ERROR ("No supported color codes found");
        return DC1394_FAILURE;
    }

    // no color specified.. pick one that is supported
    if (color == -1 || color == 0) {
        color = codings.codings[0];
        char *tmp = dc1394_colorcode_int_to_string (color);
        ERROR ("Color code not specified, setting to [%s]", tmp);
        free (tmp);
    }

    // verify color choice
    for (int i=0; i<codings.num; ++i) {
        if (codings.codings[i] == color) break;
        if (i == codings.num-1) {
            ERROR ("Specified scalable color coding not supported by this video mode");
            return DC1394_FAILURE;
        }
    }

    // get scalable video parameters
    uint32_t h, v, h_unit, v_unit, h_unit_pos, v_unit_pos;
    dc1394_format7_get_max_image_size (perls_cam->cam, mode, &h, &v);
    dc1394_format7_get_unit_size (perls_cam->cam, mode, &h_unit, &v_unit);
    dc1394_format7_get_unit_position (perls_cam->cam, mode, &h_unit_pos, &v_unit_pos);

    // set temporary packet size
    uint32_t smallest, largest;
    dc1394_format7_get_packet_parameters (perls_cam->cam, mode, &smallest, &largest);
    //status = dc1394_format7_set_packet_size (perls_cam->cam, mode, largest);
    status = dc1394_format7_set_roi (perls_cam->cam, mode, color, -2, 0, 0, h, v);
    DC1394_ERR_RTN (status, "Pre-setting packet size");

    // set color coding
    status = dc1394_format7_set_color_coding (perls_cam->cam, mode, color);
    DC1394_ERR_RTN (status, "Setting color coding in FORMAT7");

    if (left == -1 && width >= 0 && width < h) left = (h-width)/2;
    else left = 0;
    if (top == -1 && height >= 0 && height < v) top = (v-height)/2;
    else top = 0;
    if (width == -1) width = h-left;
    if (height == -1) height = v-top;
    
    int flag = 0;
    if (left % h_unit_pos != 0) { left = left - left%h_unit_pos; flag = 1;}
    if (top % v_unit_pos != 0) { top = top - top%v_unit_pos; flag = 1;}
    if (width % h_unit != 0) { width = width - width%h_unit; flag = 1;}
    if (height % v_unit != 0) { height = height - height%v_unit; flag = 1;}
    if (left + width > h || top + height > v) {
        ERROR ("Image dimensions illegal, max = %dx%d\n", h,v);
        return DC1394_FAILURE;
    }
    if (flag) printf ("Note: Modified misaligned image dimensions\n");

    status = dc1394_format7_set_image_size (perls_cam->cam, mode, width, height);
    DC1394_ERR_RTN (status, "Can't set image size");
    status = dc1394_format7_set_image_position (perls_cam->cam, mode, left, top);
    DC1394_ERR_RTN (status, "Can't set image position");

    // set packet size (below max and in unit steps)
    if (framerate < 0) framerate = 1000;
    uint32_t unit_bytes, max_bytes;
    dc1394_format7_get_packet_parameters (perls_cam->cam, mode, &unit_bytes, &max_bytes);

    dc1394speed_t speed;
    uint32_t packet_size;
    uint32_t bits;
    dc1394_get_color_coding_bit_size (color, &bits);
    dc1394_video_get_iso_speed (perls_cam->cam, &speed);
    double iso_cycles = 1000;
    switch (speed) {
        case DC1394_ISO_SPEED_100: iso_cycles = 1000.0; break;
        case DC1394_ISO_SPEED_200: iso_cycles = 2000.0; break;
        case DC1394_ISO_SPEED_400: iso_cycles = 4000.0; break;
        case DC1394_ISO_SPEED_800: iso_cycles = 8000.0; break;
        case DC1394_ISO_SPEED_1600: iso_cycles = 16000.0; break;
        case DC1394_ISO_SPEED_3200: iso_cycles = 32000.0; break;
    }

    double goal_packet_size = framerate * height * width * (bits/8.0) / iso_cycles * FORMAT7_BUS_MULTIPLIER;
    double max_fps = max_bytes * iso_cycles / height / width / (bits/8.0) / FORMAT7_BUS_MULTIPLIER;

    packet_size = (int) goal_packet_size;
    packet_size = packet_size - packet_size%unit_bytes;
    if (packet_size > max_bytes) {
        printf ("Note: Framerate maxed at %4.1f fps\n", max_fps);
        packet_size = max_bytes;
    }

    // set ROI
    status = dc1394_format7_set_packet_size (perls_cam->cam, mode, packet_size);
    DC1394_ERR_RTN (status, "Cannot set ROI for scalable video");

    status = dc1394_format7_set_roi (perls_cam->cam, mode, color, packet_size, left, top, width, height);
    DC1394_ERR_RTN (status, "Cannot set ROI for scalable video");

    // Set frame rate control register to auto (this may not be necessary here)
    uint32_t value=0;
    value = 0xC3000000; 
    status = dc1394_set_control_register (perls_cam->cam, PERLS_DC1394_CAMERA_REGISTER_FRAME_RATE_CONTROL, value);
    DC1394_ERR_RTN (status, "Cannot set the frame rate register value");

    return status;
}

/**
 * Starts the video transmission, i.e. data starts transferring and image buffer
 * starts getting filled
 */
dc1394error_t
perls_dc1394_camera_start_video_transmission (PERLS_dc1394_camera_t *perls_cam)
{
    dc1394error_t status;
    status = dc1394_capture_setup (perls_cam->cam, 4, DC1394_CAPTURE_FLAGS_DEFAULT);
    DC1394_ERR_RTN (status, "capture setup");
   
    // start data transmission
    status = dc1394_video_set_transmission (perls_cam->cam, DC1394_ON);
    DC1394_ERR_RTN (status, "enabling video transmission");
    return status;
}
 
/**
 * Stops the video transmission
 */
dc1394error_t
perls_dc1394_camera_stop_video_transmission (PERLS_dc1394_camera_t *perls_cam)
{
    dc1394error_t status;
    status = dc1394_video_set_transmission (perls_cam->cam, DC1394_OFF);
    DC1394_ERR_RTN (status, "disabling video transmission");

    // release capture resources
    status=dc1394_capture_stop (perls_cam->cam);
    DC1394_ERR_RTN (status, "Could not stop capture");

    //dc1394_camera_free (cam);

    return status;
}

/**
 * Cleans up allocated camera.
 */
void
perls_dc1394_camera_camera_free (PERLS_dc1394_camera_t *perls_cam)
{
  dc1394_camera_free (perls_cam->cam);
  dc1394_free (perls_cam->dc1394);
}
 
/**
 * These functions allow the user to set camera settings
 * Returns error code "DC1394_SUCCESS" on success and
 * "DC1394_FAILURE" on failure.
 */
void
check_range (PERLS_dc1394_camera_t *perls_cam, const char *name, const char * unit, uint32_t offset_base)
{
    // read register for setting range
    uint32_t rd;
    float min, max;
    dc1394error_t err = dc1394_get_control_register (perls_cam->cam, offset_base, &rd);
    memcpy (&min, &rd, sizeof (rd));
    err += dc1394_get_control_register (perls_cam->cam, offset_base + 0x4, &rd);
    memcpy (&max, &rd, sizeof (rd));

    if (err == DC1394_SUCCESS)
        printf ("%s range: [%g to %g] %s\n", name, min, max, unit);
}

dc1394error_t
perls_dc1394_camera_set_brightness_manual (PERLS_dc1394_camera_t* perls_cam, double value)
{
    dc1394error_t err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_BRIGHTNESS, DC1394_ON);
    err += dc1394_feature_set_mode (perls_cam->cam, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL);
    err += dc1394_feature_set_absolute_value (perls_cam->cam, DC1394_FEATURE_BRIGHTNESS, value);
    return err;
}

dc1394error_t
perls_dc1394_camera_set_brightness_auto (PERLS_dc1394_camera_t *perls_cam)
{
    dc1394error_t err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_BRIGHTNESS, DC1394_OFF);
    return err;
}

dc1394error_t
perls_dc1394_camera_set_brightness (PERLS_dc1394_camera_t* perls_cam, int manual, double value)
{
    dc1394error_t status;
    if (manual == 0)
        status = perls_dc1394_camera_set_brightness_manual (perls_cam, 0);
    else
        status = perls_dc1394_camera_set_brightness_manual (perls_cam, value);

    check_range (perls_cam, "brightness", "%", 0x930);

    return status;
}

dc1394error_t
perls_dc1394_camera_set_exposure_manual (PERLS_dc1394_camera_t *perls_cam, double value)
{
    dc1394error_t err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_EXPOSURE, DC1394_ON);
    err += dc1394_feature_set_mode (perls_cam->cam, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_MANUAL);
    err += dc1394_feature_set_absolute_value (perls_cam->cam, DC1394_FEATURE_EXPOSURE, value);
    return err;
}

dc1394error_t
perls_dc1394_camera_set_exposure_mode (PERLS_dc1394_camera_t *perls_cam, char mode[64])
{
    dc1394error_t err;
    if (0==strcmp (mode, "auto")) {
        err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_EXPOSURE, DC1394_ON);
        err += dc1394_feature_set_mode (perls_cam->cam, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_AUTO);
    }
    else if (0==strcmp (mode, "manual")) {
        err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_EXPOSURE, DC1394_ON);
        err += dc1394_feature_set_mode (perls_cam->cam, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_MANUAL);
    }
    else if (0==strcmp (mode, "off"))
        err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_EXPOSURE, DC1394_OFF);
    else
        err = DC1394_FAILURE;

    return err;
}

dc1394error_t
perls_dc1394_camera_set_exposure (PERLS_dc1394_camera_t* perls_cam, int manual, double value)
{
    dc1394error_t status;
    if (manual == 0)
        status = perls_dc1394_camera_set_exposure_mode (perls_cam, "auto");
    else {
        status = perls_dc1394_camera_set_exposure_mode (perls_cam, "manual");
        status += perls_dc1394_camera_set_exposure_manual (perls_cam, value);
    }

    check_range (perls_cam, "exposure", "EV", 0x900);

    return status;
}

dc1394error_t
perls_dc1394_camera_set_gamma_manual (PERLS_dc1394_camera_t *perls_cam, double value)
{
    dc1394error_t err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_GAMMA, DC1394_ON);
    err += dc1394_feature_set_mode (perls_cam->cam, DC1394_FEATURE_GAMMA, DC1394_FEATURE_MODE_MANUAL);
    err += dc1394_feature_set_absolute_value (perls_cam->cam, DC1394_FEATURE_GAMMA, value);
    return err;
}

dc1394error_t
perls_dc1394_camera_set_gamma_mode (PERLS_dc1394_camera_t *perls_cam, char mode[64])
{
    dc1394error_t err;
    
    if (0==strcmp (mode, "manual")) {
        err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_GAMMA, DC1394_ON);
        err += dc1394_feature_set_mode (perls_cam->cam, DC1394_FEATURE_GAMMA, DC1394_FEATURE_MODE_MANUAL);    
    }
    else if (0==strcmp (mode, "off"))
        err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_GAMMA, DC1394_OFF);
    else
        err = DC1394_FAILURE;

    return err;
}

dc1394error_t
perls_dc1394_camera_set_gamma (PERLS_dc1394_camera_t* perls_cam, int manual, double value)
{
    dc1394error_t status;
    if (manual == 0)
        status = perls_dc1394_camera_set_gamma_mode (perls_cam, "off");
    else {
        status = perls_dc1394_camera_set_gamma_mode (perls_cam, "manual");
        status += perls_dc1394_camera_set_gamma_manual (perls_cam, value);
    }

    check_range (perls_cam, "gamma", "", 0x940);

    return status;
}

dc1394error_t
perls_dc1394_camera_set_shutter_manual (PERLS_dc1394_camera_t *perls_cam, double value)
{
    dc1394error_t err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_SHUTTER, DC1394_ON);
    err += dc1394_feature_set_mode (perls_cam->cam, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
    err += dc1394_feature_set_absolute_value (perls_cam->cam, DC1394_FEATURE_SHUTTER, value);
    return err;
}

dc1394error_t
perls_dc1394_camera_set_shutter_mode (PERLS_dc1394_camera_t *perls_cam, char mode[64])
{
    dc1394error_t err;
    
    if (0==strcmp (mode, "auto")) {
        err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_SHUTTER, DC1394_ON);
        err += dc1394_feature_set_mode (perls_cam->cam, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO);
    }
    else if (0==strcmp(mode, "manual")) {
        err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_SHUTTER, DC1394_ON);
        err += dc1394_feature_set_mode (perls_cam->cam, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
    }
    else if (0==strcmp(mode, "off"))
        err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_SHUTTER, DC1394_OFF);
    else
        err = DC1394_FAILURE;
    
    return err;
}

dc1394error_t
perls_dc1394_camera_set_shutter (PERLS_dc1394_camera_t* perls_cam, int manual, double value)
{
    dc1394error_t status;
    if (manual == 0)
        status = perls_dc1394_camera_set_shutter_mode (perls_cam, "auto");
    else {
        status = perls_dc1394_camera_set_shutter_mode (perls_cam, "manual");
        status += perls_dc1394_camera_set_shutter_manual (perls_cam, value);
    }

    check_range (perls_cam, "shutter", "seconds", 0x910);

    return status;
}

dc1394error_t
perls_dc1394_camera_set_gain_manual (PERLS_dc1394_camera_t *perls_cam, int value)
{
    dc1394error_t err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_GAIN, DC1394_ON);
    err += dc1394_feature_set_mode (perls_cam->cam, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);
    err += dc1394_feature_set_absolute_value (perls_cam->cam, DC1394_FEATURE_GAIN, value);
    return err;
}

dc1394error_t
perls_dc1394_camera_set_gain_mode (PERLS_dc1394_camera_t *perls_cam, char mode[64])
{
    dc1394error_t err;
    if (0==strcmp (mode, "auto")) {
        err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_GAIN, DC1394_ON);
        err += dc1394_feature_set_mode (perls_cam->cam, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO);
    }
    else if (0==strcmp (mode, "manual")) {
        err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_GAIN, DC1394_ON);
        err += dc1394_feature_set_mode (perls_cam->cam, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);
    }
    else if (0==strcmp(mode, "off"))
        err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_GAIN, DC1394_OFF);
    else
        err = DC1394_FAILURE;
    
    return err;
}

dc1394error_t
perls_dc1394_camera_set_gain (PERLS_dc1394_camera_t* perls_cam, int manual, double value)
{
    dc1394error_t status;
    if (manual == 0)
        status = perls_dc1394_camera_set_gain_mode (perls_cam, "auto");
    else {
        status = perls_dc1394_camera_set_gain_mode (perls_cam, "manual");
        status += perls_dc1394_camera_set_gain_manual (perls_cam, value);
    }

    check_range (perls_cam, "gain", "dB", 0x920);

    return status;
}

dc1394error_t
perls_dc1394_camera_set_whitebalance_manual (PERLS_dc1394_camera_t *perls_cam, int valueBlue, int valueRed)
{
    dc1394error_t err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_WHITE_BALANCE, DC1394_ON);
    err += dc1394_feature_set_mode (perls_cam->cam, DC1394_FEATURE_WHITE_BALANCE, DC1394_FEATURE_MODE_MANUAL);
    err += dc1394_feature_whitebalance_set_value (perls_cam->cam, valueBlue, valueRed);
    return err;
}

dc1394error_t
perls_dc1394_camera_set_whitebalance_auto (PERLS_dc1394_camera_t *perls_cam)
{
    dc1394error_t err = dc1394_feature_set_power (perls_cam->cam, DC1394_FEATURE_WHITE_BALANCE, DC1394_OFF);
    return err;
}

dc1394error_t
perls_dc1394_camera_set_whitebalance (PERLS_dc1394_camera_t* perls_cam, int manual, int blue, int red)
{
    dc1394error_t status;
    if (manual == 0)
        status = perls_dc1394_camera_set_whitebalance_auto (perls_cam);
    else
        status = perls_dc1394_camera_set_whitebalance_manual (perls_cam, blue, red);

    return status;
}

/**
 * Embeds the timestamp coming from dc1394 library 
 * to the first few bytes of the image frame.
 */
dc1394error_t
perls_dc1394_camera_embed_timestamp_into_frame (PERLS_dc1394_camera_t *perls_cam)
{
    uint32_t value=0;
    dc1394error_t status = dc1394_get_control_register (perls_cam->cam, PERLS_DC1394_CAMERA_REGISTER_FRAME_TIMESTAMP, &value);
    DC1394_ERR_RTN (status, "The timestamp function is not available");

    // check presence field for this feature
    if (!(value & 0x80000000))
        printf ("Camera does not support the timestamp feature - upgrade firmware\n");

    // force enable timestamp bit
    value = value | 0x1U;
    status = dc1394_set_control_register (perls_cam->cam, PERLS_DC1394_CAMERA_REGISTER_FRAME_TIMESTAMP, value);
    DC1394_ERR_RTN (status, "Cannot set the timestamp register value");
    return status;
}


/*
 * Conversions from enumerations to strings
*/
char *
dc1394_videomode_int_to_string (int n)
{
    switch (n) {
        case DC1394_VIDEO_MODE_160x120_YUV444:      return strdup ("DC1394_VIDEO_MODE_160x120_YUV444");
        case DC1394_VIDEO_MODE_320x240_YUV422:      return strdup ("DC1394_VIDEO_MODE_320x240_YUV422");
        case DC1394_VIDEO_MODE_640x480_YUV411:      return strdup ("DC1394_VIDEO_MODE_640x480_YUV411");
        case DC1394_VIDEO_MODE_640x480_YUV422:      return strdup ("DC1394_VIDEO_MODE_640x480_YUV422");
        case DC1394_VIDEO_MODE_640x480_RGB8:        return strdup ("DC1394_VIDEO_MODE_640x480_RGB8");
        case DC1394_VIDEO_MODE_640x480_MONO8:       return strdup ("DC1394_VIDEO_MODE_640x480_MONO8");
        case DC1394_VIDEO_MODE_640x480_MONO16:      return strdup ("DC1394_VIDEO_MODE_640x480_MONO16");
        case DC1394_VIDEO_MODE_800x600_YUV422:      return strdup ("DC1394_VIDEO_MODE_800x600_YUV422");
        case DC1394_VIDEO_MODE_800x600_RGB8:        return strdup ("DC1394_VIDEO_MODE_800x600_RGB8");
        case DC1394_VIDEO_MODE_800x600_MONO8:       return strdup ("DC1394_VIDEO_MODE_800x600_MONO8");
        case DC1394_VIDEO_MODE_1024x768_YUV422:     return strdup ("DC1394_VIDEO_MODE_1024x768_YUV422");
        case DC1394_VIDEO_MODE_1024x768_RGB8:       return strdup ("DC1394_VIDEO_MODE_1024x768_RGB8");
        case DC1394_VIDEO_MODE_1024x768_MONO8:      return strdup ("DC1394_VIDEO_MODE_1024x768_MONO8");
        case DC1394_VIDEO_MODE_800x600_MONO16:      return strdup ("DC1394_VIDEO_MODE_800x600_MONO16");
        case DC1394_VIDEO_MODE_1024x768_MONO16:     return strdup ("DC1394_VIDEO_MODE_1024x768_MONO16");
        case DC1394_VIDEO_MODE_1280x960_YUV422:     return strdup ("DC1394_VIDEO_MODE_1280x960_YUV422");
        case DC1394_VIDEO_MODE_1280x960_RGB8:       return strdup ("DC1394_VIDEO_MODE_1280x960_RGB8");
        case DC1394_VIDEO_MODE_1280x960_MONO8:      return strdup ("DC1394_VIDEO_MODE_1280x960_MONO8");
        case DC1394_VIDEO_MODE_1600x1200_YUV422:    return strdup ("DC1394_VIDEO_MODE_1600x1200_YUV422");
        case DC1394_VIDEO_MODE_1600x1200_RGB8:      return strdup ("DC1394_VIDEO_MODE_1600x1200_RGB8");
        case DC1394_VIDEO_MODE_1600x1200_MONO8:     return strdup ("DC1394_VIDEO_MODE_1600x1200_MONO8");
        case DC1394_VIDEO_MODE_1280x960_MONO16:     return strdup ("DC1394_VIDEO_MODE_1280x960_MONO16");
        case DC1394_VIDEO_MODE_1600x1200_MONO16:    return strdup ("DC1394_VIDEO_MODE_1600x1200_MONO16");
        case DC1394_VIDEO_MODE_EXIF:                return strdup ("DC1394_VIDEO_MODE_EXIF");
        case DC1394_VIDEO_MODE_FORMAT7_0:           return strdup ("DC1394_VIDEO_MODE_FORMAT7_0");
        case DC1394_VIDEO_MODE_FORMAT7_1:           return strdup ("DC1394_VIDEO_MODE_FORMAT7_1");
        case DC1394_VIDEO_MODE_FORMAT7_2:           return strdup ("DC1394_VIDEO_MODE_FORMAT7_2");
        case DC1394_VIDEO_MODE_FORMAT7_3:           return strdup ("DC1394_VIDEO_MODE_FORMAT7_3");
        case DC1394_VIDEO_MODE_FORMAT7_4:           return strdup ("DC1394_VIDEO_MODE_FORMAT7_4");
        case DC1394_VIDEO_MODE_FORMAT7_5:           return strdup ("DC1394_VIDEO_MODE_FORMAT7_5");
        case DC1394_VIDEO_MODE_FORMAT7_6:           return strdup ("DC1394_VIDEO_MODE_FORMAT7_6");
        case DC1394_VIDEO_MODE_FORMAT7_7:            return strdup ("DC1394_VIDEO_MODE_FORMAT7_7");
        default: return NULL;
    }
}

char *
dc1394_colorcode_int_to_string (int n)
{
    switch (n) {
        case DC1394_COLOR_CODING_MONO8:     return strdup ("DC1394_COLOR_CODING_MONO8");
        case DC1394_COLOR_CODING_YUV411:    return strdup ("DC1394_COLOR_CODING_YUV411");
        case DC1394_COLOR_CODING_YUV422:    return strdup ("DC1394_COLOR_CODING_YUV422");
        case DC1394_COLOR_CODING_YUV444:    return strdup ("DC1394_COLOR_CODING_YUV444");
        case DC1394_COLOR_CODING_RGB8:      return strdup ("DC1394_COLOR_CODING_RGB8");
        case DC1394_COLOR_CODING_MONO16:    return strdup ("DC1394_COLOR_CODING_MONO16");
        case DC1394_COLOR_CODING_RGB16:     return strdup ("DC1394_COLOR_CODING_RGB16");
        case DC1394_COLOR_CODING_MONO16S:   return strdup ("DC1394_COLOR_CODING_MONO16S");
        case DC1394_COLOR_CODING_RGB16S:    return strdup ("DC1394_COLOR_CODING_RGB16S");
        case DC1394_COLOR_CODING_RAW8:      return strdup ("DC1394_COLOR_CODING_RAW8");
        case DC1394_COLOR_CODING_RAW16:      return strdup ("DC1394_COLOR_CODING_RAW16");
        default: return NULL;
    }
}

char *
dc1394_framerate_int_to_string (int n)
{
    switch (n) {
        case DC1394_FRAMERATE_1_875:    return strdup ("DC1394_FRAMERATE_1_875");
        case DC1394_FRAMERATE_3_75:     return strdup ("DC1394_FRAMERATE_3_75");
        case DC1394_FRAMERATE_7_5:      return strdup ("DC1394_FRAMERATE_7_5");
        case DC1394_FRAMERATE_15:       return strdup ("DC1394_FRAMERATE_15");
        case DC1394_FRAMERATE_30:       return strdup ("DC1394_FRAMERATE_30");
        case DC1394_FRAMERATE_60:       return strdup ("DC1394_FRAMERATE_60");
        case DC1394_FRAMERATE_120:      return strdup ("DC1394_FRAMERATE_120");
        case DC1394_FRAMERATE_240:      return strdup ("DC1394_FRAMERATE_240");
        default: return NULL;
    }
}

double
dc1394_framerate_int_to_double (int n)
{
    switch (n) {
        case DC1394_FRAMERATE_1_875:    return 1.875;
        case DC1394_FRAMERATE_3_75:     return 3.75;
        case DC1394_FRAMERATE_7_5:      return 7.5;
        case DC1394_FRAMERATE_15:       return 15;
        case DC1394_FRAMERATE_30:       return 30;
        case DC1394_FRAMERATE_60:       return 60;
        case DC1394_FRAMERATE_120:      return 120;
        case DC1394_FRAMERATE_240:      return 240;
        default: return -1;
    }
}

int
dc1394_videomode_string_to_int (const char *s)
{
    if (!strcmp (s, "DC1394_VIDEO_MODE_160x120_YUV444")) return DC1394_VIDEO_MODE_160x120_YUV444;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_320x240_YUV422")) return DC1394_VIDEO_MODE_320x240_YUV422;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_640x480_YUV411")) return DC1394_VIDEO_MODE_640x480_YUV411;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_640x480_YUV422")) return DC1394_VIDEO_MODE_640x480_YUV422;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_640x480_RGB8")) return DC1394_VIDEO_MODE_640x480_RGB8;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_640x480_MONO8")) return DC1394_VIDEO_MODE_640x480_MONO8;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_640x480_MONO16")) return DC1394_VIDEO_MODE_640x480_MONO16;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_800x600_YUV422")) return DC1394_VIDEO_MODE_800x600_YUV422;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_800x600_RGB8")) return DC1394_VIDEO_MODE_800x600_RGB8;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_800x600_MONO8")) return DC1394_VIDEO_MODE_800x600_MONO8;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_1024x768_YUV422")) return DC1394_VIDEO_MODE_1024x768_YUV422;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_1024x768_RGB8")) return DC1394_VIDEO_MODE_1024x768_RGB8;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_1024x768_MONO8")) return DC1394_VIDEO_MODE_1024x768_MONO8;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_800x600_MONO16")) return DC1394_VIDEO_MODE_800x600_MONO16;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_1024x768_MONO16")) return DC1394_VIDEO_MODE_1024x768_MONO16;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_1280x960_YUV422")) return DC1394_VIDEO_MODE_1280x960_YUV422;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_1280x960_RGB8")) return DC1394_VIDEO_MODE_1280x960_RGB8;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_1280x960_MONO8")) return DC1394_VIDEO_MODE_1280x960_MONO8;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_1600x1200_YUV422")) return DC1394_VIDEO_MODE_1600x1200_YUV422;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_1600x1200_RGB8")) return DC1394_VIDEO_MODE_1600x1200_RGB8;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_1600x1200_MONO8")) return DC1394_VIDEO_MODE_1600x1200_MONO8;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_1280x960_MONO16")) return DC1394_VIDEO_MODE_1280x960_MONO16;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_1600x1200_MONO16")) return DC1394_VIDEO_MODE_1600x1200_MONO16;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_EXIF")) return DC1394_VIDEO_MODE_EXIF;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_FORMAT7_0")) return DC1394_VIDEO_MODE_FORMAT7_0;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_FORMAT7_1")) return DC1394_VIDEO_MODE_FORMAT7_1;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_FORMAT7_2")) return DC1394_VIDEO_MODE_FORMAT7_2;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_FORMAT7_3")) return DC1394_VIDEO_MODE_FORMAT7_3;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_FORMAT7_4")) return DC1394_VIDEO_MODE_FORMAT7_4;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_FORMAT7_5")) return DC1394_VIDEO_MODE_FORMAT7_5;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_FORMAT7_6")) return DC1394_VIDEO_MODE_FORMAT7_6;
    else if (!strcmp (s, "DC1394_VIDEO_MODE_FORMAT7_7")) return DC1394_VIDEO_MODE_FORMAT7_7;
    else return -1;
}

int
dc1394_colorcode_string_to_int (const char *s)
{
    if (!strcmp (s, "DC1394_COLOR_CODING_MONO8")) return DC1394_COLOR_CODING_MONO8;
    else if (!strcmp (s, "DC1394_COLOR_CODING_YUV411")) return DC1394_COLOR_CODING_YUV411;
    else if (!strcmp (s, "DC1394_COLOR_CODING_YUV422")) return DC1394_COLOR_CODING_YUV422;
    else if (!strcmp (s, "DC1394_COLOR_CODING_YUV444")) return DC1394_COLOR_CODING_YUV444;
    else if (!strcmp (s, "DC1394_COLOR_CODING_RGB8")) return DC1394_COLOR_CODING_RGB8;
    else if (!strcmp (s, "DC1394_COLOR_CODING_MONO16")) return DC1394_COLOR_CODING_MONO16;
    else if (!strcmp (s, "DC1394_COLOR_CODING_RGB16")) return DC1394_COLOR_CODING_RGB16;
    else if (!strcmp (s, "DC1394_COLOR_CODING_MONO16S")) return DC1394_COLOR_CODING_MONO16S;
    else if (!strcmp (s, "DC1394_COLOR_CODING_RGB16S")) return DC1394_COLOR_CODING_RGB16S;
    else if (!strcmp (s, "DC1394_COLOR_CODING_RAW8")) return DC1394_COLOR_CODING_RAW8;
    else if (!strcmp (s, "DC1394_COLOR_CODING_RAW16")) return DC1394_COLOR_CODING_RAW16;
    else return -1;
}

int
dc1394_framerate_string_to_int (const char *s)
{
    if (!strcmp (s, "DC1394_FRAMERATE_1_875")) return DC1394_FRAMERATE_1_875;
    else if (!strcmp (s, "DC1394_FRAMERATE_3_75")) return DC1394_FRAMERATE_3_75;
    else if (!strcmp (s, "DC1394_FRAMERATE_7_5")) return DC1394_FRAMERATE_7_5;
    else if (!strcmp (s, "DC1394_FRAMERATE_15")) return DC1394_FRAMERATE_15;
    else if (!strcmp (s, "DC1394_FRAMERATE_30")) return DC1394_FRAMERATE_30;
    else if (!strcmp (s, "DC1394_FRAMERATE_60")) return DC1394_FRAMERATE_60;
    else if (!strcmp (s, "DC1394_FRAMERATE_120")) return DC1394_FRAMERATE_120;
    else if (!strcmp (s, "DC1394_FRAMERATE_240")) return DC1394_FRAMERATE_240;
    else return -1;
}

