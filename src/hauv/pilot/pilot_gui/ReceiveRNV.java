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

public class ReceiveRNV implements LCMSubscriber
{	
    public double time_dvl;
    public double x_dvl;
    public double y_dvl;
    public double heading_dvl;
	public double depth_dvl;
	public double distance_dvl;

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            hauv.bs_rnv_2_t rnv = new hauv.bs_rnv_2_t(ins);
            time_dvl = rnv.time;
            x_dvl = -1*rnv.vertical;
            y_dvl = rnv.horizontal;
            heading_dvl = rnv.heading;
			depth_dvl = rnv.depth;	
			distance_dvl = rnv.distance;		

            //System.out.println("This is a message from the Listener.");
            //System.out.println("X_DVL: "+x_dvl+" Y_DVL: "+y_dvl+" Heading_DVL: "+heading_dvl);
        } catch (IOException ex) {
            System.out.println("Error decoding message: "+ex);
        }
    }
}  

