#include "evologics_modem.hpp"


int loop_exit;


void on_heartbeat(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const heartbeat_t *hb, Evologics_Modem* ev) 
{
    ev->keep_alive(); 
}

void on_auv_status(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const auv_status_t *status, Evologics_Modem* ev) 
{
    ev->send_status(status);    
}

// a callback with no automatic decoding
void on_lcm(const lcm::ReceiveBuffer* rbuf, const std::string& channel, Evologics_Modem* ev) 
{
    char dest_channel[64];
    memset(dest_channel, 0, 64);
    strcpy(dest_channel, channel.c_str());
    strcat(dest_channel, ".1");
    ev->evo->send_lcm_data((unsigned char *)rbuf->data, rbuf->data_size, 1, dest_channel);
}

Evologics_Modem::Evologics_Modem()
{
    lcm = new lcm::LCM();
}

int Evologics_Modem::keep_alive()
{
    if(keep_alive_count == 15)
    {
        keep_alive_count = 0;
        evo->send_command("+++AT?S\r");
    }
    else
        keep_alive_count++;

    return 1;
}

int Evologics_Modem::send_status(const auv_status_t *status)
{
    // the message has been decoded so we will need to re encode it
    // we will also update the id field
    auv_status_t s;
    memcpy(&s, status, sizeof(auv_status_t));
    
    s.target_id = local_address;
    
    char target_channel[16];
	memset(target_channel, 0, 16);
    sprintf(target_channel, "AUV_STATUS.%d", local_address);
    int d_size = s.getEncodedSize();
    unsigned char *d = (unsigned char *)malloc(d_size);
    s.encode(d, 0, s.getEncodedSize());
    evo->send_lcm_data(d, d_size, usbl_address, target_channel);
    free(d);
    
    return 1;
}
    

int Evologics_Modem::load_config(char *program_name)
{
    BotParam *param = NULL;
    param = bot_param_new_from_server (lcm->getUnderlyingLCM(), 1);
    if(param == NULL)
        return 0;
        
    char rootkey[64];        
    char key[128];
    sprintf (rootkey, "sensors.%s", program_name);


    // read the initial config
    sprintf(key, "%s.device", rootkey);
    device = bot_param_get_str_or_fail(param, key);

	sprintf(key, "%s.baud", rootkey);
    baud = bot_param_get_int_or_fail(param, key);

    sprintf(key, "%s.parity", rootkey);
    parity = bot_param_get_str_or_fail(param, key);  
    
	sprintf(key, "%s.usbl_address", rootkey);
    usbl_address = bot_param_get_int_or_fail(param, key);

	sprintf(key, "%s.local_address", rootkey);
    local_address = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.lcm", rootkey);
    lcm_channels = NULL;
    lcm_channels = bot_param_get_str_array_alloc(param, key);
    
    sprintf(key, "%s.gain", rootkey);
    gain = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.source_level", rootkey);
    source_level = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.auto_gain", rootkey);
    auto_gain = bot_param_get_boolean_or_fail(param, key);
    
    return 1;
}        



int Evologics_Modem::init()
{
    // Open the serial port
    evo_fd = serial_open(device, serial_translate_speed(baud), serial_translate_parity(parity), 1);    
    //serial_set_canonical(state.fd, '\r', '\n');
    serial_set_noncanonical(evo_fd, 1, 0);
    
    // Create the Evologics object
    evo = new Evologics(evo_fd, '\r', lcm);
    
    // subscribe to the relevant streams
    
    // put the modem in a known state
    evo->send_command("+++ATZ1\r");
    
    char cmd[64];
    sprintf(cmd, "+++AT!L%d\r", source_level);
    evo->send_command(cmd);
    sprintf(cmd, "+++AT!G%d\r", gain);
    evo->send_command(cmd);

    if(auto_gain)
        evo->send_command("+++AT!LC1\r");
    
    evo->send_command("+++ATZ1\r");
    
    // now to force the settings that require a listen mode
    evo->send_command("+++ATN\r");      // noise mode
    evo->send_command("+++ATA\r");      // listen state
    
    keep_alive_count = 0;
    
    // Subscribe to LCM messages
    lcm->subscribeFunction("HEARTBEAT_1HZ", on_heartbeat, this);
    lcm->subscribeFunction("AUV_STATUS", on_auv_status, this);
    
    int i = 0;
    while(lcm_channels[i] != NULL)
    {
        lcm->subscribeFunction(lcm_channels[i], on_lcm, this);
        i++;
      
    }
    
    return 1;
}

int Evologics_Modem::process()
{
    int lcm_fd = lcm->getFileno();
    fd_set rfds;
    char buf[MAX_BUF_LEN];
    
    while(!loop_exit)
    {
        FD_ZERO (&rfds);
        FD_SET (lcm_fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        memset(buf, 0, MAX_BUF_LEN);
        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &timeout);
        if(ret > 0)
        {
            if(FD_ISSET(lcm_fd, &rfds))
                lcm->handle();
        }
        
    }
    delete evo;
    
    return 1;
}


void signal_handler(int sig)
{
    loop_exit = 1;
}

// Main entry point
int main(int argc, char **argv)
{
    // install the exit handler
    loop_exit = 0;
    signal(SIGINT, signal_handler);
    Evologics_Modem *modem = new Evologics_Modem;
    modem->load_config(basename((argv[0])));
    modem->init();
    
    modem->process();
    
    delete modem;
    return 1;
}