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
import hauv.vehicle_state_t;

public class ReceiveNVG implements LCMSubscriber
{	
    public double time_navigator;
    public double x_navigator;
    public double y_navigator;
    public double heading_navigator;

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            hauv.vehicle_state_t nvg = new hauv.vehicle_state_t(ins);
            time_navigator = nvg.time;
            x_navigator = nvg.x;
            y_navigator = nvg.y;
            heading_navigator = nvg.heading;
            //System.out.println("X_NVG: "+x_navigator+" Y_NVG: "+y_navigator+" Heading_NVG: "+heading_navigator);
        } catch (IOException ex) {
            System.out.println("Error decoding message: "+ex);
        }
    } 
}  
