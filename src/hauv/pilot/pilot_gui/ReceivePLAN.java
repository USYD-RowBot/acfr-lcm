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
import hauv.vehicle_plan_t;

public class ReceivePLAN implements LCMSubscriber
{	
    public double time;
    public boolean stop;
    public boolean holdStation;
    public boolean heading;
    public boolean broadcast;
    public double headingOffset;
    public int npoints;
    public double[][] waypoints;
    public double depth;

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            hauv.vehicle_plan_t plan = new hauv.vehicle_plan_t(ins);
            time = plan.time;
            stop = plan.stop;
            heading = plan.heading;
            headingOffset = plan.headingOffset;
            holdStation = plan.holdStation;
            broadcast = plan.broadcast;
            npoints = plan.npoints;
            waypoints = plan.waypoints;
            depth = plan.depth;
        } catch (IOException ex) {
            System.out.println("Error decoding message: "+ex);
        }
    } 
}  
