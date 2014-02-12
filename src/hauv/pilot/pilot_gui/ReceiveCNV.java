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
import hauv.bs_cnv_t;
import hauv.vehicle_state_t;

public class ReceiveCNV implements LCMSubscriber
{	
    public double time1_cnv;
    public double time2_cnv;
    public double x_cnv;
    public double y_cnv;
    public double z_cnv;
    public double heading_cnv;

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            hauv.bs_cnv_t cnv = new hauv.bs_cnv_t(ins);
            time1_cnv = cnv.time_received;
            time2_cnv = cnv.time_nav;      
            x_cnv = cnv.x;
            y_cnv = cnv.y;
            z_cnv = cnv.z; 
            heading_cnv = cnv.heading;

            //System.out.println("This is a message from the Listener.");
            //System.out.println("X_CNV: "+x_cnv+" Y_CNV: "+y_cnv+" Heading_CNV: "+heading_cnv);
        } catch (IOException ex) {
            System.out.println("Error decoding message: "+ex);
        }
    }
}  

