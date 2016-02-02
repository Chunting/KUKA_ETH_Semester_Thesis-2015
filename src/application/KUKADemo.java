package application;

/*
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.net.Socket;
*/

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import task.MWAtask;
import task.Scraping5;
import task.ScrapeSmooth;
import task.Whittling3;
import task.Sawing1;
import matlabcontrol.*;


public class KUKADemo extends RoboticsAPIApplication {
	
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	private Tool scraper;
	
	private MWAtask myTask;
	
	private int imp = 0;
	
	int UI_focus;
	int dir;
	
	private static final double v_taxingRel = 0.25;		// relative velocity for non task movements [1]
	
	private ObjectFrame F_flange;		//flange frame
	private ObjectFrame F_root;			//root frame
		
	//standBy position in joint space§
	double a1 = Math.toRadians(0);		
	double a2 = Math.toRadians(45);
	double a3 = Math.toRadians(0);
	double a4 = Math.toRadians(-45);
	double a5 = Math.toRadians(0);
	double a61 = Math.toRadians(60);
	double a62 = Math.toRadians(-20);
	double a7 = Math.toRadians(0);
		
	JointPosition J_standBy1 = new JointPosition(a1,a2,a3,a4,a5,a61,a7); // scraping standby
	JointPosition J_standBy2 = new JointPosition(a1,a2,a3,a4,a5,a62,a7); //whittling standby
	JointPosition J_standBy;
	
	double A1 = Math.toRadians(40);		// joint angles
	double A2 = Math.toRadians(68);
	double A3 = Math.toRadians(0);
	double A4 = Math.toRadians(-94);
	double A5 = Math.toRadians(42);
	double A6 = Math.toRadians(102);
	double A7 = Math.toRadians(-35);
	
	JointPosition J_micro =  new JointPosition(A1,A2,A3,A4,A5,A6,A7);
	
	
	/********************User Input**************************/
	String UI_task	 	= "Scraping";
	// select out of: "Scraping", "Whittling","ScrapingImp","WhittlingImp","Sawing","Scrapebone","ScrapeSmooth"
	
	String UI_tool 		= "MiniA"; //;"MedB";
	 // select out of: "MedA", "MedB", "SmallA", "SmallB","SmallC","LargeA","MiniA","MiniE"
	
	int UI_numIntervals	= 1;
	// between 1 and 5
	
	int UI_numCycles 	= 20;
	// between 1 and 500
	
	int UI_force 		= 70;
	//  force in z direction, between 10 and 80 [N]
	
	/********************************************************/
	
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
		
		F_flange = lbr_iiwa_14_R820_1.getFlange();		//flange frame
		F_root = lbr_iiwa_14_R820_1.getRootFrame();		//root frame
	}
	
	
	public void run() {

		switch (UI_tool){
			case "MedA":	scraper = getApplicationData().createFromTemplate("ScraperMediumA");
			        break;
			case "MedB":	scraper = getApplicationData().createFromTemplate("ScraperMediumB");
			        break;
			case "SmallA":	scraper = getApplicationData().createFromTemplate("ScraperSmallA");
			        break;
			case "SmallB":	scraper = getApplicationData().createFromTemplate("ScraperSmallB");
			       	break;
			case "SmallC":	scraper = getApplicationData().createFromTemplate("ScraperSmallC");
	                break;
			case "LargeA":	scraper = getApplicationData().createFromTemplate("ScraperLargeA");
            		break;			
			case "MiniA":	scraper = getApplicationData().createFromTemplate("MiniA");
    		        break;
			case "MiniE":	scraper = getApplicationData().createFromTemplate("MiniE");
	                break;
		}

		assert UI_numIntervals	>= 1 	&& UI_numIntervals 	<= 5;
		assert UI_numCycles 	>= 1 	&& UI_numCycles 	<= 500;
		assert UI_force			>= 10	&& UI_force 		<= 80;
        
		
		switch (UI_task){
			case "Scraping":
					myTask = new Scraping5(lbr_iiwa_14_R820_1, scraper, UI_force);
			        imp = 0;
			        J_standBy = J_standBy1 ;
					break;
					
			case "Whittling":
					myTask = new Whittling3(lbr_iiwa_14_R820_1, scraper, UI_force);
			        imp = 0;
			        J_standBy = J_standBy2 ;					
			        break;

			case "Sawing":
					myTask = new Sawing1(lbr_iiwa_14_R820_1, scraper, UI_force, UI_numCycles);
					imp = 1;
	                J_standBy = J_standBy2 ;	
	                UI_numCycles = 1;
			        break;
      
			case "ScrapeSmooth":
					myTask = new ScrapeSmooth(lbr_iiwa_14_R820_1, scraper, UI_force);
					imp = 1;
                    J_standBy = J_standBy1 ;	
                    break;
                    
            default:
            		System.out.println("invalid task");
            		System.exit(0); 
		}
		
		
		scraper.attachTo(F_flange);

		//Creating file name parameter
		DateFormat d = new SimpleDateFormat("dd-MM-yyyy-HH-mm-ss");
	    Date date = new Date();
	    String dat = d.format(date);	
		
	    System.out.println("going to standBy position");
		scraper.move(ptp(J_standBy).setJointVelocityRel(v_taxingRel));
	    
		for (int i_inter=1; i_inter <= UI_numIntervals; i_inter++) {
			
			// go to standby position
			System.out.println("going to standBy position");
			scraper.move(ptp(J_standBy).setJointVelocityRel(v_taxingRel));
			
			// creating the filename using the required parameters
			String ni = 	String.format("%d",i_inter);
			String filename = dat +"_" + UI_task + "_" + UI_tool + "_" + UI_force + "_" + UI_numCycles + "_" + ni + ".log" ;
			
			// create and start data recording
			DataRecorder rec = new DataRecorder(); //data recording 
			rec.setFileName(filename);
			rec.setSampleRate(100);
			rec.addCartesianForce(F_flange,F_root);
			rec.addCurrentCartesianPositionXYZ(F_flange, F_root);
			
			rec.enable();
			rec.startRecording();
			System.out.println("recording activated");
			if(rec.isFileAvailable())
				System.out.println("data file available");
			System.out.println(rec.getURL().toString());
			
			// start cycling
			System.out.println("start cycling the task");
			myTask.firstCycle = true;
			myTask.lastCycle = false;
			
			// one interval experiment 
			for (int i_cycle = 1; i_cycle <= UI_numCycles; i_cycle++) {
				System.out.println(String.format("cycle # %1$d", i_cycle));
				
				if(i_cycle == UI_numCycles )
				myTask.lastCycle = true;
				
				myTask.goCycle(imp);
				
			}
			
		
			scraper.move(ptp(J_standBy).setJointVelocityRel(v_taxingRel));
			
			// stop recording
			rec.stopRecording();
			System.out.println("data recording stopped");	
		
			/*
			// go to mounting position
			System.out.println("Going to mounting position");	
			MountingPosition MP = new MountingPosition();
			MP.runApplication();
			*/
			// microscopy();
			
			//go to microscope
			//dir = getApplicationUI().displayModalDialog( ApplicationDialogType.QUESTION, "Do you want to go to microscope?", "Yes", "No");
			//if(dir == 0)
			
			//{
		    // microscopy();
			//}
			
		}
		 
			
		// move to standBy position:
		System.out.println("going to standBy position");
		scraper.move(ptp(J_standBy).setJointVelocityRel(v_taxingRel));
		
	}

	   public static double matlab(int i) throws MatlabConnectionException, MatlabInvocationException {
	    	
	    	MatlabProxyFactoryOptions options = new MatlabProxyFactoryOptions.Builder().setUsePreviouslyControlledSession(true).setMatlabLocation(null).build();
	        MatlabProxyFactory factory = new MatlabProxyFactory(options);
	        MatlabProxy proxy = factory.getProxy();
	     //   proxy.disconnect();
	     //   proxy = factory.getProxy(); // this won't open a new Matlab session
	        
	        if(i==0)
	        proxy.eval("fm = Focus(0,vid)");
	        else if (i==1)
	        proxy.eval("fm = Focus(1,vid)");
	        else if (i==2)
	        proxy.eval("fm = Focus(2,vid)");
	        else if (i==3)
	        proxy.eval("fm = Focus(3,vid)");
	        else if (i==4){
	        proxy.eval("opengl software");	
	        proxy.eval("caminit");
	        proxy.disconnect();
	        return 0 ;
	        }
	        else if (i==5){
	        proxy.eval("stoppreview(vid)");
	        proxy.eval("closepreview(vid)");
	        proxy.eval("clear");
	        proxy.disconnect();
	        return 0 ;
	        }
	        	
	        	
	        	
	        double fm = ((double[]) proxy.returningEval("fm", 1)[0])[0];
	        System.out.println("Focus measure: " + fm);
	        
	        //Disconnect the proxy from MATLAB
	        proxy.disconnect();
	        return fm;
	    }
		
		private void microscopy() {
			
		/*	// set-up the impedance mode
			System.out.println("Setting impedance mode");
			CartesianImpedanceControlMode microImp = new CartesianImpedanceControlMode();
				
			microImp.parametrize(CartDOF.X,CartDOF.Y).setStiffness(4500);  // max 5000
			microImp.parametrize(CartDOF.Z).setStiffness(100);
			microImp.parametrize(CartDOF.TRANSL).setDamping(0.7);
		
			microImp.parametrize(CartDOF.ROT).setStiffness(290);		// max 300
			microImp.parametrize(CartDOF.ROT).setDamping(0.9);
		 
			microImp.setNullSpaceStiffness(1000);
		
			microImp.setMaxControlForce(10, 10, 20, 10, 10, 10, true);
			*/
			double z = 72.5;
			Frame F_micro2 = new Frame(F_root,100, 301, 165+z,Math.toRadians(-130),Math.toRadians(-90),Math.toRadians(130));
			
			Frame F_micro = new Frame(F_root);
			F_micro.setTransformationFromParent(lbr_iiwa_14_R820_1.getForwardKinematic(J_micro));
			Frame F_micro1 = F_micro.copy(); 
			F_micro1.setX(F_micro.getX()+30);
			F_micro1.setY(F_micro.getY()-30);
			F_micro1.setZ(F_micro.getZ()+25+z);
		
		    scraper.move(lin(F_micro1).setJointVelocityRel(0.25));
		    scraper.move(lin(F_micro2).setJointVelocityRel(0.1));
		    
		    scraper.move(linRel(-5.7,-12,8.2,F_root).setJointVelocityRel(0.01)); // focusing between 173 to 176
		    //scraper.move(linRel(0,0,0,0,0,Math.toRadians(-45),F_root).setJointVelocityRel(0.2));
		    /*
		     int dir = getApplicationUI().displayModalDialog( ApplicationDialogType.QUESTION, "Do you want to autofocus?", "Yes", "No");
			if(dir == 1)
			{
		     return;
			}
		   
		    NavigatorCloseloop navigator = new NavigatorCloseloop(scraper, F_root);
			try {
				navigator.activate();
			} catch (MatlabConnectionException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (MatlabInvocationException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}	
		    */
			System.out.println("Navigator starting");
			
			Navigator navigator1 = new Navigator(scraper, F_root, lbr_iiwa_14_R820_1);
			navigator1.activate();	// terminate by ENTER
			
			
			// getApplicationControl().pause();
			
			scraper.move(linRel(50,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
			scraper.move(linRel(100,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.1));
			
			// go to standby position
		
			//scraper.move(ptp(J_standBy).setJointVelocityRel(v_taxingRel));
		}
		
		
		public static void main(String[] args) {
			KUKADemo app = new KUKADemo();
			app.runApplication();
		}
}