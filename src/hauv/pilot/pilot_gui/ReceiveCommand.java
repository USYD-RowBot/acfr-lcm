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

public class ReceiveCommand implements LCMSubscriber
{	

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            hauv.bs_rnv_t rnv = new hauv.bs_rnv_t(ins);
            System.out.println("Horizontal Position is "+rnv.horizontal);
            System.out.println("Vertical Position is "+rnv.vertical);
            //Write the latest nav. to a text file
            //BufferedWriter out = new BufferedWriter(new FileWriter("navRecord.txt"));
            //out.write(""+rnv.horizontal+" "+rnv.vertical);
            //out.close();
        } catch (IOException ex) {
            System.out.println("Error decoding message: "+ex);
        }
    }

    public static void main(String args[])
    {
        LCM myLCM = LCM.getSingleton();	
        myLCM.subscribe("HAUV_BS_RNV", new ReceiveCommand());
	
        // Sleep forever: if we quit, so will the LCM thread.
        while (true) 
            {
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException ex) {
                }
            }
    } 
}  
