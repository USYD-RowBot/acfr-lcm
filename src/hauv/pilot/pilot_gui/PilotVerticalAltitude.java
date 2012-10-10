package pilot_gui;
import lcm.lcm.*;
import java.util.Scanner;
import java.util.LinkedList;
import java.io.*;
import java.lang.Math;
import hauv.pl_raw_t;
import hauv.pl_san_t;
import hauv.pl_ghp_t;
import hauv.pl_gbp_t;
import hauv.pl_rbo_t;
import hauv.bs_nvg_t;
import hauv.bs_rnv_2_t;
import hauv.bs_dvl_t;
import hauv.vehicle_state_t;
//import hauv.vehicle_command_t;
import hauv.vehicle_plan_t;

public class PilotVerticalAltitude implements Runnable {

    // Right now this includes everything EXCEPT the heading correction
    // Run in terminal window and print current status to screen (e.g - current status:
    // mission, current status: holding station at .... )

    //Declare a few variables
    static Thread t = null;
    static LCM myLCM = LCM.getSingleton();
    public static int flag = 0; // Used for starting/killing the waypoint-issuing thread
    public double timeSpec = 2e5; // Max allowed time descrepancy between RNV and NAVIGATOR
    public static int clickCount = 0; // May need to implement if we get multiple listeners
    static double thetaDrift = 0; // Used in both the main() and run() modules
    //static double heading_cmd = 0;
    public double xdiff = 0.75; //Capture radius in x
    public double ydiff = 0.75; //Capture radius in y

    public int rnvListSize = 10; //Max number of RNV messages to store in list

    //These correspond to our three listeners
    static ReceiveRNV rnv = new ReceiveRNV();
    static ReceiveNVG nvg = new ReceiveNVG();
    static ReceivePLAN plan = new ReceivePLAN();

    public void run() {

        //suspend the HAUV mission
        hauv.pl_sus_t sus = new hauv.pl_sus_t();
        sus.time = System.nanoTime();
       	myLCM.publish ("HAUV_PL_SUS", sus);

        //get a nav update and compute the global location of the waypoints
        int timeCheck = 0;	
        double time_dvl = rnv.time_dvl;
        double x_dvl = rnv.x_dvl; 
        double y_dvl = rnv.y_dvl; 
        double y_hold = y_dvl; // this will be the fixed sway position (new)
        double depth = rnv.depth_dvl; //new field added for this version of Pilot
        double altitude = rnv.distance_dvl; //new field added for this version of Pilot
        double heading_dvl = rnv.heading_dvl;
        double time_nav = nvg.time_navigator;
        double x_nav = nvg.x_navigator; 
        double y_nav = nvg.y_navigator; 
        double heading_nav = nvg.heading_navigator; 			
  
        /*
          while (timeCheck == 0) {
          time_dvl = rnv.time_dvl;
          x_dvl = rnv.x_dvl; 
          y_dvl = rnv.y_dvl; 
          heading_dvl = rnv.heading_dvl;
          time_nav = nvg.time_navigator;
          x_nav = nvg.x_navigator; 
          y_nav = nvg.y_navigator; 
          heading_nav = nvg.heading_navigator; 			
          if (Math.abs(time_dvl-time_nav)<timeSpec) {
          timeCheck = 1;
          }
          try {
          //Delay for () milliseconds
          Thread.sleep(50); 
          } catch (InterruptedException ex) {
          }
          }
        */

        /////////////////////////////////////////////////////////
        // ADD CURRENT NAV TO THE WAYPOINTS FROM THE PLAN PACKAGE
        double des_depth = plan.depth;
        int npoints = plan.npoints;
        //double[][] waypts = plan.waypoints;

        System.out.println(plan.waypoints[0][0]+" "+plan.waypoints[0][1]+" "+plan.waypoints[1][0]+" "+plan.waypoints[1][1]+" "); //+waypts[2][0]+" "+waypts[2][1]+" "+waypts[3][0]+" "+waypts[3][1]);	
	
        double[][] wayptsGlobal = new double[npoints][2];
        for (int irow = 0; irow < npoints; irow++){
            //wayptsGlobal[irow][0] = plan.waypoints[irow][0] + x_nav; // use these only when navigator
            //wayptsGlobal[irow][1] = plan.waypoints[irow][1] + y_nav; // is turned on

            wayptsGlobal[irow][0] = plan.waypoints[irow][0] + altitude; //plan.waypoints[irow][0] + x_dvl; previously
            wayptsGlobal[irow][1] = -plan.waypoints[irow][1] + x_dvl; //plan.waypoints[irow][1] + y_dvl; previously
            // sway command will simply be y_dvl
        }
        y_hold = y_dvl;
        /////////////////////////////////////////////////////////

        //Now that we have the waypoints, issue them sequentially to the backseat driver
        int irow = 0;
        int nextWaypt = 0;
        double xwaypt;
        double ywaypt;
        double xDelta = 0.0;
        double yDelta = 0.0;
        double tDelta = 0.0;
        double xDeltaPrime;
        double yDeltaPrime;
        //double thetaDrift; This is now defined above to allow for heading reset
        double xcommand;
        double ycommand;
        int sendCount = 0;

        hauv.pl_gbp_t gbp = new hauv.pl_gbp_t();
        ////////////////////////////////////////////////
        // New List for storing RNV data
        LinkedList<Double> rnvTime = new LinkedList<Double>();
        LinkedList<Double> rnvX  = new LinkedList<Double>();
        LinkedList<Double> rnvY = new LinkedList<Double>();
        LinkedList<Double> rnvT = new LinkedList<Double>();
        ////////////////////////////////////////////////

        while (flag == 1) {	 //alternatively, try "reset==1"
            if (irow == npoints){
                irow = 0;
            }
            //Read in the next waypoint		
            xwaypt = wayptsGlobal[irow][0];
            ywaypt = wayptsGlobal[irow][1];
		
            while (nextWaypt == 0 && flag == 1) {

                //////////////////////////////////////////////////////////////////////
                //Add the newest RNV message to the list of messages
                double timeNewest = rnv.time_dvl;
                double xNewest = rnv.x_dvl;
                double yNewest = rnv.y_dvl;
                double tNewest = rnv.heading_dvl;
                double timeBest = timeNewest;
                double xBest = xNewest;
                double yBest = yNewest;
                double tBest = tNewest;
                /*
                  if (rnvTime.size() < rnvListSize) {
                  rnvTime.add(timeNewest);
                  rnvX.add(xNewest);
                  rnvY.add(yNewest);
                  rnvT.add(tNewest);
                  }
                  else {
                  double oldestMessage = rnvTime.getFirst();
                  int oldestIndex = 0;
                  for (int i = 1; i < rnvTime.size(); i++) {
                  if (rnvTime.get(i) < oldestMessage) {
                  oldestMessage = rnvTime.get(i);
                  oldestIndex = i;
                  }
                  }
                  rnvTime.set(oldestIndex,timeNewest);
                  rnvX.set(oldestIndex,xNewest);
                  rnvY.set(oldestIndex,yNewest);
                  rnvT.set(oldestIndex,tNewest);
                  System.out.println("Difference between oldest and newest: "+Math.abs(oldestMessage - timeNewest));
                  //System.out.println("Oldest Time: "+oldestMessage+" Newest Time: "+timeNewest);
                  }
                */
                //Choose the RNV message which best matches the NAVIGATOR message
                time_nav = nvg.time_navigator;
                time_dvl = timeNewest;
                /*
                  for (int i = 0; i < rnvTime.size(); i++) {
                  if (Math.abs(rnvTime.get(i) - time_nav) < Math.abs(time_dvl-time_nav)) {
                  time_dvl = rnvTime.get(i);
                  timeBest = rnvTime.get(i);
                  xBest = rnvX.get(i);
                  yBest = rnvY.get(i);
                  tBest = rnvT.get(i);
                  System.out.println("The most recent message isn't the best!");
                  }
                  }
                */
                // Since we aren't running the navigator, keep xBest, yBest, etc. set to the newest values 
      
                ///////////////////////////////////////////////////////////////////////

                //System.out.println("Time diff: " + Math.abs(time_dvl-time_nav) + " " + rnvBest.time_dvl + " " + nvg.time_navigator);

                //if (Math.abs(time_dvl-time_nav)<timeSpec) {		
				System.out.println("Difference between Navigator and DVL: "+Math.abs(time_dvl-time_nav));	
				//Get a navigation update
				x_dvl = xBest; 
				y_dvl = yBest; 
				altitude = rnv.distance_dvl;
				heading_dvl = tBest; 
				x_nav = nvg.x_navigator; 
				y_nav = nvg.y_navigator; 
				heading_nav = nvg.heading_navigator; 
				System.out.println("DVL: "+x_dvl+" "+y_dvl+" " +heading_dvl+ " NAV: "+x_nav+" "+y_nav+" "+heading_nav);	//used for testing the listener		

                /*
				//Determine the current distance from the waypoint
				xDelta = xwaypt - x_nav;
				yDelta = ywaypt - y_nav;

				//Transform the relative waypoint into the dvl frame
				thetaDrift = heading_nav - heading_dvl; //Make sure these are both in radians!!
				xDeltaPrime = Math.cos(thetaDrift)*xDelta + Math.sin(thetaDrift)*yDelta;
				yDeltaPrime = -Math.sin(thetaDrift)*xDelta + Math.cos(thetaDrift)*yDelta;

				//Compute the global waypoint in the dvl frame
				xcommand = x_dvl + xDeltaPrime;
				ycommand = y_dvl + yDeltaPrime;				
                */
				double a = 0.1; // 0.001;
				double xNewDelta, yNewDelta, tNewDelta;
				tNewDelta = heading_nav - heading_dvl;
				if (tNewDelta > Math.PI) tNewDelta -= Math.PI;
				if (tNewDelta < -Math.PI) tNewDelta += Math.PI;

				tDelta = tDelta*(1-a) + tNewDelta*a;
				if (tDelta > Math.PI) tDelta -= Math.PI;
				if (tDelta < -Math.PI) tDelta += Math.PI;

                xNewDelta = x_nav - (Math.cos(-tDelta)*x_dvl - Math.sin(-tDelta)*y_dvl);
                yNewDelta = y_nav - (Math.sin(-tDelta)*x_dvl + Math.cos(-tDelta)*y_dvl);

				System.out.println("New delta: " + xNewDelta + " " + yNewDelta + " " + tNewDelta);
				xDelta = xDelta*(1-a) + xNewDelta*a;
				yDelta = yDelta*(1-a) + yNewDelta*a;
				System.out.println("Delta: " + xDelta + " " + yDelta + " " + tDelta);

				// wp_dvl = R(wp_nav-delta)
                //xcommand = Math.cos(tDelta)*(xwaypt-xDelta) - Math.sin(tDelta)*(ywaypt-yDelta);
                //ycommand = Math.sin(tDelta)*(xwaypt-xDelta) + Math.cos(tDelta)*(ywaypt-yDelta);
                //thetaDrift = tDelta;
    
                /////////////////////////////////////////////////////////////////////////////////
                // Just use the values from the DVL: (for use when navigator is turned off)
                /////////////////////////////////////////////////////////////////////////////////
                xcommand = xwaypt;
                ycommand = ywaypt;
                thetaDrift = 0;
                /////////////////////////////////////////////////////////////////////////////////

				//Issue the waypoint command using backseat driver				
				if (sendCount == 0) {
					//gbp = new hauv.pl_gbp_t();
					gbp.time = System.nanoTime();	    	
					gbp.x = y_hold; //ycommand previously
					gbp.y = -ycommand; //-xcommand previously
					gbp.relative_bearing = 0;//-thetaDrift; //Change back to zero if errors occur
					gbp.is_depth = false; //true
					gbp.depth_altitude = xcommand; //des_depth previously
					sendCount = 0;
				}
				myLCM.publish ("HAUV_PL_SUS", sus);
				myLCM.publish ("HAUV_PL_GBP", gbp);

				//Print useful waypoint information to the terminal
				//System.out.println("Navigator X: "+x_nav+" Y: "+y_nav+" T: "+heading_nav+" DVL X: "+x_dvl+" Y: "+y_dvl+" T: "+heading_dvl);
				System.out.println("x_dvl: "+x_dvl+" ywaypt: "+ywaypt+" y_dvl: "+y_dvl+" y_hold: "+y_hold+" altitude: "+altitude+" xwaypt: "+xwaypt);					
				//System.out.println("Navigator Waypoint X: "+xwaypt+" Y: "+ywaypt+" DVL Waypoint X: "+xcommand+" Y: "+ycommand+" irow: "+irow); 
				//System.out.println("Waypoints are: "+waypts[0][0]+" "+waypts[1][0]+" "+waypts[1][0]+" "+waypts[1][1]+" "+waypts[2][0]+" "+waypts[2][1]+" "+waypts[3][0]+" "+waypts[3][1]);

				////////////////////////////////////////////////////////////////////////
				// CHECK TO SEE IF WE ARE HOLDING STATION BEFORE ADVANCING THE WAYPOINT
				////////////////////////////////////////////////////////////////////////
				boolean holdStation = plan.holdStation;
				if (holdStation == true) {
					System.out.println("Holding station.");
				}
				else {
					//Check to see if vehicle is within capture radius
					if((Math.abs(x_dvl-ywaypt) < xdiff) && 
					   (Math.abs(y_dvl-y_hold) < ydiff) &&
                       (Math.abs(altitude-xwaypt) < xdiff))
                        //if((Math.abs(x_dvl+gbp.y) < xdiff) && 
                        //   (Math.abs(y_dvl-gbp.x) < ydiff))
                        {
                            nextWaypt = 1;
                            irow++;
                            System.out.println("Within capture radius!");
                        }
                    //}
                    ////////////////////////////////////////////////////
                }
                try {
                    //Delay for () milliseconds
                    Thread.sleep(500); 
                } catch (InterruptedException ex) {
                }
            }
            //Reset the inner loop
            nextWaypt = 0;
            sendCount = 0;
			
            try {
                //Delay for () milliseconds
                Thread.sleep(100); 
            } catch (InterruptedException ex) {
            }
        }
    }

    public static void main(String args[]) {

        //Start the listener only once
        myLCM.subscribe("HAUV_BS_RNV_2", rnv);
        myLCM.subscribe("HAUV_VEHICLE_STATE", nvg);
        myLCM.subscribe("HAUV_VEHICLE_PLAN", plan);

        boolean stop = false;
        boolean heading = false;
        boolean broadcast = false;
        //boolean	stopPrevious = stop;
        //boolean headingPrevious = heading;
	
        while (true) {
            // Update two of our boolean variables
            stop = plan.stop;
            heading = plan.heading;
            broadcast = plan.broadcast; 
            ///////////////////////////////////////////////////////////////////////////////////////
            //if (stop == true) { // If there has been a change to the STOP variable
			if (stop == true) {
				System.out.println("Stop Mode.");
				//Document the occurence of a new click
				//clickCount++;
				if (t != null) {
					flag = 0; //set the flag to zero long enough that the while loop will catch it and shut down	
					//System.out.println("Flag is set to zero");
					try {
                        //Delay for () milliseconds
                        Thread.sleep(1000); 
					} catch (InterruptedException ex) {
					} 
				}	
				hauv.pl_res_t res = new hauv.pl_res_t();
				res.time = System.nanoTime();
				myLCM.publish ("HAUV_PL_RES", res);

				//stopPrevious = true;
			}	
			if (broadcast == true) {
				System.out.println("Survey Mode.");

				// Stop the current mission
				if (t != null) {
					flag = 0; //set the flag to zero long enough that the while loop will catch it and shut down	
					//System.out.println("Flag is set to zero");
					try {
                        //Delay for () milliseconds
                        Thread.sleep(1000); 
					} catch (InterruptedException ex) {
					} 
				} 	

				//Start the thread which issues waypoint commands
				flag = 1;	
				PilotVerticalAltitude p = new PilotVerticalAltitude();
				t = new Thread(p);
				t.start();

				//suspend the HAUV mission
				hauv.pl_sus_t sus = new hauv.pl_sus_t();
				sus.time = System.nanoTime();
                myLCM.publish ("HAUV_PL_SUS", sus);

				//stopPrevious = stop;
            }
            //}	
            /////////////////////////////////////////////////////////////////
            //if (heading != headingPrevious) { // If there has been a change to the HEADING variable
            if (heading == true) {
                System.out.println("Heading Correction Mode.");
                //issue a new heading command	
                double heading_cmd = plan.headingOffset;
                //suspend the HAUV mission
                hauv.pl_sus_t sus = new hauv.pl_sus_t();
                sus.time = System.nanoTime();
                myLCM.publish ("HAUV_PL_SUS", sus);
                //issue the heading command
                hauv.pl_rbo_t rbo = new hauv.pl_rbo_t();
                rbo.time = System.nanoTime();
                rbo.angle_offset = -thetaDrift + heading_cmd;
                myLCM.publish("HAUV_PL_RBO", rbo);			
				
                //heading = false;
            }
            //}
            try {
                //Delay for () milliseconds
                Thread.sleep(100); 
            } catch (InterruptedException ex) {
            }
    	}
    }
}

