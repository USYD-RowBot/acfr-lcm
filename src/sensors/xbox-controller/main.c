/**
 * @author Nick Carlevaris-Bianco - carlevar@umich.edu
 *
 * @file xbox-controller/main.c
 *
 * @brief LCM driver for Xbox 360 wireless and wired controllers
 *
 * @details
 * 
 * This driver works with both the linux kernel module xpad and the user space
 * Xboxdrv (http://pingus.seul.org/~grumbel/xboxdrv/)
 *
 * By default we use the Xboxdrv driver because it publishes a "back to neutral"
 * event on wireless disconnect and because it allows us to use Dbus to check
 * the wireless receiver status for disconnects. Without at least one of these
 * two features we cannot differentiate between holding a joystick forward and
 * a disconnect while moving forward. This is dangerous for any application
 * were you may move out of range. Therefore it is probably best to install
 * Xboxdrv unless you are using a wired controller.
 *
 * USING XBOXDRV
 * Xboxdrv (http://pingus.seul.org/~grumbel/xboxdrv/)
 * 
 * To install (ubuntu 10.04):
 * sudo add-apt-repository ppa:grumbel/ppa
 * sudo apt-get update
 * sudo apt-get install xboxdrv
 *
 * Turn off the kernel module that usually reads xbox as a joystick
 * $ sudo rmmod xpad (temporary)
 * add "blacklist xpad" to /etc/modprobe.d/blacklist.conf (permanent)
 *
 * Load additional kernel modules
 * $ (sudo) modprobe uinput (temporary)
 * $ (sudo) modprobe joydev (temporary)
 * add "uinput" & "joydev" to /etc/modules (permanent)
 *
 * Adjust udev rules so that you can access uinput
 * add something like with your group
 * KERNEL=="uinput", MODE="0660", GROUP="yourgroup" 
 * or
 * KERNEL=="uinput", MODE="0666"
 * and
 * ATTR{idVendor}=="045e", GROUP="yourgroup", MODE="0666" (idVendor is found using lsusb)
 * to /etc/udev/rules.d/99-xbox.rules
 *
 * run xboxdrv
 * $ xboxdrv --daemon --dpad-as-button
 * or to have it start up automaticaly install the init.d script
 * TODO DOESNT WORK CURRENTLY: it works once you are logged in but on startup will not work
 * might need to write a service instead, or use rc.local ?
 * $ sudo cp ~/perls/install/xboxdrv /etc/init.d/.
 * $ sudo update-rd.d xboxdrv defaults
 */


#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <dbus/dbus.h>
#include <pthread.h>

#include <linux/input.h>
#include <linux/joystick.h>

#include "perls-lcmtypes/senlcm_xbox_controller_t.h"
#include "perls-lcmtypes/perllcm_xbox_command_t.h"

#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"

#define WIRELESS_XPAD_NAME "Xbox 360 Wireless Receiver"
#define WIRELESS_XBOXDRV_NAME "Xbox Gamepad (userspace driver)"
#define WIRED_XBOXDRV_NAME "Microsoft X-Box 360 pad"

#define READ_TIMEOUT_USEC 100000; // 1/10 of a second

typedef enum _xbox_controller_driver_t xbox_controller_driver_t;
enum _xbox_controller_driver_t {
    XBOXDRV = 1,      // must be installed (default)
    XPAD = 2          // default kernel module
};

//----------------------------------------------------------------------------------
// STATE STRUCTURE 
//----------------------------------------------------------------------------------
typedef struct _state_t state_t;
struct _state_t {
    int done;
    int is_daemon;
    int print;
    
    lcm_t *lcm;
    const char *lcm_channel;
    const char *cmd_channel;
    
    // connection to joystick
    int fd;
    // joystick info
    int n_axes;
    int n_buttons;
    char js_name[128];
    xbox_controller_driver_t driver;
    
    //dbus connection
    DBusConnection *dbus_conn;
    
    // xbox joystick button state
    senlcm_xbox_controller_t xbox_ctrl_state;
    
    // xbox command state for rumble/LED
    perllcm_xbox_command_t xbox_cmd_state;
    
    // constantly check if we have received message from joystick recently
    int js_connected;
};

//Init the state structure to zero
state_t state = {0};

//----------------------------------------------------------------------------------
// Update state structure for controller setup
//----------------------------------------------------------------------------------
void
state_cmd_cb (const lcm_recv_buf_t *rbuf, const char *channel, 
        const perllcm_xbox_command_t *msg, void *user)
{
    // copy over -- send dbus commands
    state.xbox_cmd_state.utime = msg->utime;
    state.xbox_cmd_state.left_rumble = msg->left_rumble;
    state.xbox_cmd_state.right_rumble = msg->right_rumble;
    state.xbox_cmd_state.led_code = msg->led_code;
}


void
send_controller_command (void)
{
    DBusMessage *msg;
    DBusMessageIter args;

    // create message for rumbles
    msg = dbus_message_new_method_call ("org.seul.Xboxdrv", // target for the method call
                                      "/org/seul/Xboxdrv/ControllerSlots/0", // object to call on
                                      "org.seul.Xboxdrv.Controller", // interface to call on
                                      "SetRumble"); // method name

    dbus_message_iter_init_append (msg, &args);
    dbus_int32_t r1 = state.xbox_cmd_state.left_rumble;
    dbus_int32_t r2 = state.xbox_cmd_state.right_rumble;
    dbus_message_iter_append_basic (&args, DBUS_TYPE_INT32, &r1);
    dbus_message_iter_append_basic (&args, DBUS_TYPE_INT32, &r2);
    dbus_connection_send (state.dbus_conn, msg, NULL);

    // free message
    dbus_message_unref (msg);

    // create message for LEDs
    msg = dbus_message_new_method_call ("org.seul.Xboxdrv", // target for the method call
                                      "/org/seul/Xboxdrv/ControllerSlots/0", // object to call on
                                      "org.seul.Xboxdrv.Controller", // interface to call on
                                      "SetLed"); // method name

    dbus_message_iter_init_append (msg, &args);
    dbus_int32_t l = state.xbox_cmd_state.led_code;
    dbus_message_iter_append_basic (&args, DBUS_TYPE_INT32, &l);
    dbus_connection_send (state.dbus_conn, msg, NULL);

    // free message
    dbus_message_unref (msg);
}


//----------------------------------------------------------------------------------
// Given a joystick event update the xbox controller state 
//----------------------------------------------------------------------------------
void
update_xbox_ctrl_state (struct js_event js)
{    
    // copy joystick state pointer over from state
    senlcm_xbox_controller_t *x = &state.xbox_ctrl_state;
    
    //Switch statement maps buttons / axis number from driver to our state
    //depends on which driver
    switch (state.driver) {
    case XBOXDRV:
        switch (js.type & ~JS_EVENT_INIT) {
        case JS_EVENT_BUTTON:
            switch (js.number) {
            case 0: x->dpad_u = js.value; break;
            case 1: x->dpad_d = js.value; break;
            case 2: x->dpad_l = js.value; break;
            case 3: x->dpad_r = js.value; break;
            case 4: x->a_btn = js.value; break;
            case 5: x->b_btn = js.value; break;
            case 6: x->x_btn = js.value; break;
            case 7: x->y_btn = js.value; break;
            case 8: x->l_bump = js.value; break;
            case 9: x->r_bump = js.value; break;
            case 10: x->back_btn = js.value; break;
            case 11: x->start_btn = js.value; break;
            case 12: x->xbox_btn = js.value; break;
            case 13: x-> l_stick_btn= js.value; break;
            case 14: x-> r_stick_btn= js.value; break;
            default:
                ERROR ("uknown JS_EVENT_BUTTON event (%d)", js.number);
            }
            break;
        case JS_EVENT_AXIS:
            switch(js.number) {
            case 0: x->l_stick_x = js.value; break;
            case 1: x->l_stick_y = js.value; break; 
            case 2: x->r_stick_x = js.value; break;
            case 3: x->r_stick_y = js.value; break; 
            case 4: x->r_trig = js.value; break;
            case 5: x->l_trig = js.value; break;
            default:
                ERROR ("uknown JS_EVENT_AXIS event (%d)", js.number);
            }
            break;
        case JS_EVENT_INIT:
            if (!state.is_daemon && state.print)
                printf("INIT EVENT \n");
            break;
        }
        break;
    
    case XPAD: //built in kernel driver
        switch (js.type & ~JS_EVENT_INIT) {
        case JS_EVENT_BUTTON:
            switch (js.number) {
            case 0: x->a_btn = js.value; break;
            case 1: x->b_btn = js.value; break;
            case 2: x->x_btn = js.value; break;
            case 3: x->y_btn = js.value; break;
            case 4: x->l_bump = js.value; break;
            case 5: x->r_bump = js.value; break;
            case 6: x->start_btn = js.value; break;
            case 7: x->xbox_btn = js.value; break;
            case 8: x-> l_stick_btn= js.value; break;
            case 9: x-> r_stick_btn= js.value; break;
            case 10: x->dpad_u = js.value; break;
            case 11: x->dpad_d = js.value; break;
            case 12: x->dpad_l = js.value; break;
            case 13: x->dpad_r = js.value; break;
            case 14: x->back_btn = js.value; break;
            default:
                ERROR ("uknown JS_EVENT_BUTTON event (%d)", js.number);
            }
            break;
        case JS_EVENT_AXIS:
            switch(js.number) {
            case 0: x->l_stick_x = js.value; break;
            case 1: x->l_stick_y = js.value; break;
            case 2: x->l_trig = js.value; break;
            case 3: x->r_stick_x = js.value; break;
            case 4: x->r_stick_y = js.value; break;
            case 5: x->r_trig = js.value; break;
            default:
                ERROR ("uknown JS_EVENT_AXIS event (%d)", js.number);
            }
            break;
        case JS_EVENT_INIT:
            if (!state.is_daemon && state.print)
                printf("INIT EVENT \n");
            break;
        }
        break;
    }
}

void
set_neutral (void)
{    
    senlcm_xbox_controller_t *x = &state.xbox_ctrl_state;   
    x->a_btn = 0;
    x->b_btn = 0;
    x->x_btn = 0;
    x->y_btn = 0;
    x->l_bump = 0;
    x->r_bump = 0;
    x->start_btn = 0;
    x->xbox_btn = 0;
    x-> l_stick_btn = 0;
    x-> r_stick_btn = 0;
    x->dpad_u = 0;
    x->dpad_d = 0;
    x->dpad_l = 0;
    x->dpad_r = 0;
    x->back_btn = 0;
    
    x->l_stick_x = 0;
    x->l_stick_y = 0;
    x->r_stick_x = 0;
    x->r_stick_y = 0;
    x->l_trig = SENLCM_XBOX_CONTROLLER_T_AXIS_MIN;
    x->r_trig = SENLCM_XBOX_CONTROLLER_T_AXIS_MIN;
}

//----------------------------------------------------------------------------------
// check if controller is connected over dbus
//----------------------------------------------------------------------------------
int
is_connected_dbus (void)
{
    // for dbus info see
    // $ sudo apt-get source dbus
    // http://pingus.seul.org/~grumbel/xboxdrv/xboxdrv.html
    // http://www.matthew.ath.cx/misc/dbus
    

    if (state.driver != XBOXDRV) {
        ERROR ("ERROR: Cannot check for connection over DBus unless using xboxdrv driver.");
        return 0;
    }

    DBusError dbus_err;
    DBusMessage *msg;
    DBusMessage *reply;
    DBusMessageIter args;
    DBusMessageIter iter;
    char *status;
 
    msg = dbus_message_new_method_call ("org.seul.Xboxdrv", // target for the method call
                                        "/org/seul/Xboxdrv/Daemon", // object to call on
                                        "org.seul.Xboxdrv.Daemon", // interface to call on
                                        "Status"); // method name

    dbus_message_iter_init_append (msg, &args);

    // send message and get a handle for a reply
    dbus_error_init (&dbus_err);
    reply = dbus_connection_send_with_reply_and_block (state.dbus_conn,
                                                       msg, -1, &dbus_err);
    dbus_message_get_reply_serial (reply);
    dbus_message_iter_init (reply, &iter);
    dbus_message_iter_get_basic (&iter, &status);

    // free message
    dbus_message_unref (msg);
    dbus_message_unref (reply);
    
    // WIRELESS
    // disconnected message
    //SLOT  CFG  NCFG    USBID    USBPATH  NAME
    //   0    0     1      -         -
    //   -             045e:0719  002:003  Xbox 360 Wireless Receiver for Windows
    //   -             045e:0719  002:003  Xbox 360 Wireless Receiver for Windows
    //   -             045e:0719  002:003  Xbox 360 Wireless Receiver for Windows
    //   -             045e:0719  002:003  Xbox 360 Wireless Receiver for Windows
    // connected message
    //SLOT  CFG  NCFG    USBID    USBPATH  NAME
    //   0    0     1  045e:0719  002:003  Xbox 360 Wireless Receiver for Windows
    //   -             045e:0719  002:003  Xbox 360 Wireless Receiver for Windows
    //   -             045e:0719  002:003  Xbox 360 Wireless Receiver for Windows
    //   -             045e:0719  002:003  Xbox 360 Wireless Receiver for Windows
    
    // WIRED
    // disconnected message
    //SLOT  CFG  NCFG    USBID    USBPATH  NAME
    //   0    0     1      -         -
    // connected message
    //SLOT  CFG  NCFG    USBID    USBPATH  NAME
    //   0    0     1  045e:028e  002:007  Controller

    // HACK
    char *line, *token, *next_token;
    line = strtok (status, "\n"); // skip the header line
    line = strtok (NULL, "\n"); // read the first line
    token = strtok_r (line, " \t", &next_token);        // SLOT
    token = strtok_r (next_token, " \t", &next_token);  // CFG
    token = strtok_r (next_token, " \t", &next_token);  // NCFG
    token = strtok_r (next_token, " \t", &next_token);  // USBID

    // token now either has the USBID of the controller or '-'

    if (strcmp (token, "-") == 0)
        return 0;
    else if (strlen (token) == 9)
        return 1;
    else {
        ERROR ("ERROR: Unexpected dbus status");
        return 0;
    }
}

//----------------------------------------------------------------------------------
// publish the xbox controller state
//----------------------------------------------------------------------------------
void
xbox_controller_publish (void)
{    
    // set timestamp at publish
    // might be republishing old state when held in on position and controller
    // timestamp is not critical
    state.xbox_ctrl_state.utime = timestamp_now();   
    senlcm_xbox_controller_t_publish(state.lcm, state.lcm_channel, &(state.xbox_ctrl_state));
}

//----------------------------------------------------------------------------------
// Called when program shuts down 
//----------------------------------------------------------------------------------
static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("\nmy_signal_handler()\n");
    if (state.done) {
        printf ("Goodbye\n");
        exit (EXIT_FAILURE);
    } 
    else
        state.done = 1;
}

void *
incoming_thread (void *data)
{
    uint64_t t = 0;

    while (!state.done) {
        lcm_handle (state.lcm);

        // only send controller commands at 20 Hz
        if (timestamp_now () - t > (1e6/20.0)) {
            send_controller_command ();
            t = timestamp_now ();
        }
    }

    // turn off rumble/leds
    state.xbox_cmd_state.left_rumble = 0;
    state.xbox_cmd_state.right_rumble = 0;
    state.xbox_cmd_state.led_code = 0;
    send_controller_command ();

    return (void *) NULL;
}

//----------------------------------------------------------------------------------
// Main 
//----------------------------------------------------------------------------------
int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // install custom signal handler
    struct sigaction act = {
        .sa_sigaction = my_signal_handler,
    };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);
    
    // Read in the command line options
    getopt_t *gopt = getopt_create ();
    
    getopt_add_description (gopt, "XBox Controller Driver");
    getopt_add_description (gopt, "Note: Commands (ie. lights/rumble) can only be changed when using the XBOXDRV driver.");
    getopt_add_string (gopt,  'd',  "device",      "/dev/input/js0",     "Location of device");
    getopt_add_string (gopt,  'c',  "channel",     "XBOX_CONTROLER",     "LCM publish channel");
    getopt_add_string (gopt,  'C',  "command",     "XBOX_COMMAND",       "LCM XBOX command channel");
    getopt_add_bool (gopt,    'D',  "daemon",       0,                   "Run as system daemon");
    getopt_add_bool (gopt,    'p',  "print",        0,                   "Print state to screen");
    getopt_add_bool (gopt,    'h',  "help",         0,                   "Display Help");

    if (!getopt_parse (gopt, argc, argv, 1)) {
        getopt_do_usage (gopt,"");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt,"");
        exit (EXIT_SUCCESS);;
    }
    //start as daemon if asked
    if (getopt_get_bool (gopt, "daemon")) {
        daemon_fork ();
        state.is_daemon = 1;
    }
    else
        state.is_daemon = 0;
    
    state.print = getopt_get_bool (gopt, "print");
    
    // initialize lcm 
    state.lcm = lcm_create (NULL);
    if (!state.lcm) {
        printf ("ERROR: lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    }    
    
    // set the lcm publish channel
    state.lcm_channel = getopt_get_string (gopt, "channel");
    
    // set the lcm command channel
    state.cmd_channel = getopt_get_string (gopt, "command");

    // Open the joystick connection
    const char *device =  getopt_get_string (gopt, "device");
    if ((state.fd = open(device, O_RDONLY)) < 0) {
        ERROR ("Failed to open device %s \n", device);
        exit (EXIT_FAILURE);
    }
    
    // Querry and print info about the joystick
    ioctl (state.fd, JSIOCGAXES, &state.n_axes);
    ioctl (state.fd, JSIOCGBUTTONS, &state.n_buttons);
    ioctl (state.fd, JSIOCGNAME(128), state.js_name);
    printf ("Joystick (%s) has %d axes and %d buttons \n",
            state.js_name, state.n_axes, state.n_buttons);
    
    // make sure that we are working with the XBOX controler or quit
    // currently supports wireless and wired controllers
    if (0 == strcmp (WIRELESS_XPAD_NAME, state.js_name)) {
        state.driver = XPAD;
        printf ("Found %s using XPAD driver \n", WIRELESS_XPAD_NAME);
        printf ("WARNING: only use this driver if you are sure you will not have wireless disconnects! \n");
    }
    else if (0 == strcmp (WIRELESS_XBOXDRV_NAME, state.js_name)) {
        state.driver = XBOXDRV;
        printf ("Found %s using XBOXDRV driver \n", WIRELESS_XBOXDRV_NAME);
    }
    else if (0 == strcmp (WIRED_XBOXDRV_NAME, state.js_name)) {
        state.driver = XBOXDRV;
        printf ("Found %s using XBOXDRV driver \n", WIRED_XBOXDRV_NAME);
    }
    else {
        ERROR ("Incorrect controller, expected one of: \n%s\n%s\n%s \nfound %s \n",
               WIRELESS_XPAD_NAME,
               WIRELESS_XBOXDRV_NAME,
               WIRED_XBOXDRV_NAME,
               state.js_name);
        exit (EXIT_FAILURE);
    }
    
    // establish dbus connection
    if (XBOXDRV == state.driver) {
        // using xboxdrv, we can actually send commands on dbus connection
        // so we subscribe to incoming LCM command messages
        perllcm_xbox_command_t_subscribe (state.lcm, state.cmd_channel, &state_cmd_cb, NULL);
    
        DBusError dbus_err;
        
        // connect to the bus
        dbus_error_init(&dbus_err);
        state.dbus_conn = dbus_bus_get (DBUS_BUS_SESSION, &dbus_err);
        if (dbus_error_is_set (&dbus_err)) {
            ERROR ("ERROR: dbus connection failed: %s", dbus_err.message);
            dbus_error_free(&dbus_err);
            exit (EXIT_FAILURE);
        }
        
        // request a name on the bus
        int ret = dbus_bus_request_name(state.dbus_conn,
                                        "perl.perls.xbox_controller", 
                                        DBUS_NAME_FLAG_REPLACE_EXISTING, 
                                        &dbus_err);
        if (dbus_error_is_set(&dbus_err)) { 
            ERROR("ERROR DBus name error %s", dbus_err.message); 
            dbus_error_free(&dbus_err);
            exit (EXIT_FAILURE);
        }
        if (DBUS_REQUEST_NAME_REPLY_PRIMARY_OWNER != ret)
            exit (EXIT_FAILURE);
    }

    // if not in daemon mode print the header to the screen
    if (!state.is_daemon && state.print) {
        printf ("\n");
        printf ("|     Left Stick    |    Right Stick    |   Triggers    |  Bumps  |      D-Pad      |           Buttons           |\n");
        printf ("|   X      Y    BTN |   X      Y    BTN |   L      R    |  L   R  |  U   D   L   R  | BCK XBX ST   A   B   X   Y  |\n");
        printf ("| ------ ------ --- | ------ ------ --- | ------ ------ | --- --- | --- --- --- --- | --- --- --- --- --- --- --- |\n");
    }
    
    // read setup
    struct js_event js;
    fd_set set;
    struct timeval tv;
    pthread_t *inc_thread = NULL;
    set_neutral ();
    
    // check for incoming lcm messages
    if (XBOXDRV == state.driver) {
        inc_thread = (pthread_t *)malloc (sizeof (pthread_t));
        pthread_create (inc_thread, 0, &incoming_thread, NULL);
    }
    
    while (!state.done) {
        
        FD_ZERO (&set);
        FD_SET (state.fd, &set);
        tv.tv_sec = 0;
        tv.tv_usec = READ_TIMEOUT_USEC;

        // try to read from joystick
        int ret = select (state.fd+1, &set, NULL, NULL, &tv);
        if (ret < 0)
            ERROR ("ERROR: select()");
        else if (ret == 0) { // Timeout    
            // if using XBOXDRV we can use dsub to make sure we are still connected
            if (XBOXDRV == state.driver) {
                if (!is_connected_dbus ())
                    set_neutral ();    // if we know its not connected set neutral
            }

            
            xbox_controller_publish ();    
        }
        else { // We have data.
        
            if (read (state.fd, &js, sizeof (js) ) != sizeof (js))
                ERROR ("ERROR: reading failed");
            
            // new data update state send
            update_xbox_ctrl_state (js);
            xbox_controller_publish ();
        }

        // if not in daemon mode print the output to the screen
        if (!state.is_daemon && state.print) {
            senlcm_xbox_controller_t *x = &state.xbox_ctrl_state;
            printf ("| %6d %6d  %1d  | %6d %6d  %1d  | %6d %6d |  %1d   %1d  |  %1d   %1d   %1d   %1d  |  %1d   %1d   %1d   %1d   %1d   %1d   %1d  |\r",
                    x->l_stick_x, x->l_stick_y, x->l_stick_btn,
                    x->r_stick_x, x->r_stick_y, x->r_stick_btn,
                    x->l_trig, x->r_trig,
                    x->l_bump, x->r_bump,
                    x->dpad_u, x->dpad_d, x->dpad_l, x->dpad_r,
                    x->back_btn, x->xbox_btn, x->start_btn,
                    x->a_btn, x->b_btn, x->x_btn, x->y_btn);
            fflush (stdout);
        }            
    }

    if (inc_thread != NULL) {
        pthread_cancel (*inc_thread);

        // turn off rumble/leds
        state.xbox_cmd_state.left_rumble = 0;
        state.xbox_cmd_state.right_rumble = 0;
        state.xbox_cmd_state.led_code = 0;
        send_controller_command ();
    }

    
    if (state.dbus_conn) {
        //dbus_connection_close (state.dbus_conn);
    }
             
    printf ("\nDone.\n");    
    exit (EXIT_SUCCESS);
}
