package pilot_gui;
import lcm.lcm.*;
import java.io.*;
import hauv.pl_raw_t;
import hauv.pl_san_t;
import hauv.pl_ghp_t;
import hauv.pl_gbp_t;
import hauv.bs_nvg_t;
import hauv.bs_rnv_t;
import hauv.bs_dvl_t;
import hauv.bs_pit_t;
import hauv.vehicle_state_t;

public class ReceivePIT implements LCMSubscriber
{	
    public double time;
    public double pitch;

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            hauv.bs_pit_t pit = new hauv.bs_pit_t(ins);
            time = pit.time;
            pitch = pit.pitch_sonar;		

            //System.out.println("This is a message from the Listener.");
            //System.out.println("Pitch: "+pitch);
        } catch (IOException ex) {
            System.out.println("Error decoding message: "+ex);
        }
    }
}  

