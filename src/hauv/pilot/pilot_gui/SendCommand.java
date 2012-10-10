package pilot_gui;
import java.io.*;
import java.util.Scanner;
import lcm.lcm.*;
import hauv.pl_san_t;
import hauv.pl_gbp_t;
import hauv.pl_ghp_t;
import hauv.pl_sus_t;
//import lcmtypes.*;

public class SendCommand
{
    public static void main(String args[])
    {
    	LCM myLCM = LCM.getSingleton();

        Scanner input = new Scanner( System.in );
        //double xcoord;
        //double ycoord;	
        //double altitude;
        double angle;
        //System.out.println( "Enter the desired x coordinate: " );
        //xcoord = input.nextDouble();
        //System.out.println( "Enter the desired y coordinate: " );
        //ycoord = input.nextDouble();
        //System.out.println( "Altitude: " );
        //altitude = input.nextDouble();
        System.out.println( "Enter the desired sonar angle: " );
        angle = input.nextDouble()/180*3.141527;
        System.out.println( "Sending command to vehicle... ");

        //suspend the HAUV mission
        hauv.pl_sus_t sus = new hauv.pl_sus_t();
        sus.time = System.nanoTime();
        myLCM.publish ("HAUV_PL_SUS", sus);

        //adjust DIDSON pitch angle
        hauv.pl_san_t san = new hauv.pl_san_t();
        san.time = System.nanoTime();        
        san.angle = angle;
        myLCM.publish ("HAUV_PL_SAN", san);
	
        /*
          int count = 0;
          while (true)
          {		
          count++;
          hauv.pl_gbp_t gbp = new hauv.pl_gbp_t();
          gbp.time = System.nanoTime();	    	
          gbp.x = xcoord;
          gbp.y = ycoord;
          gbp.relative_bearing = 0;
          gbp.is_depth = true;
          gbp.depth_altitude = 0.1;
          if (count>=10) {
          count = 0;
          }
          myLCM.publish ("HAUV_PL_SUS", sus);
          myLCM.publish ("HAUV_PL_GBP", gbp);


          //hauv.pl_ghp_t ghp = new hauv.pl_ghp_t();
          //ghp.time = System.nanoTime();	    	
          //ghp.horizontal = xcoord;
          //ghp.vertical = ycoord;
          //ghp.distance = 2.0;
          //myLCM.publish ("HAUV_PL_GHP", ghp);


          try {
          Thread.sleep(1000);
          } catch (InterruptedException ex) {
          }
          }
        */
    }
}


