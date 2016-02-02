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
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

import task.MWAtask;
import task.MountingPosition;
import task.Scrapedemo;
import task.Scraping5;
import task.ScrapeSmooth;
import task.Scrapingbone;
import task.Whittling3;
import task.Microscopy;
import task.Sawing1;
import matlabcontrol.*;


public class KUKADemo2 extends RoboticsAPIApplication {
	
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	private Tool scraper;
	
	private MWAtask myTask;
	
	private int imp = 0;
	
	int UI_focus;
	int dir;
	
	private static final double v_taxingRel = 0.8;		// relative velocity for non task movements [1]
	
	private ObjectFrame F_flange;		//flange frame
		
	//standBy position in joint space
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
	String UI_task	 	= "Scrapedemo";
	// select out of: "Scraping", "Whittling","ScrapingImp","WhittlingImp","Sawing","Scrapebone","ScrapeSmooth"
	
	String UI_tool 		= "SmallE";
	 // select out of: "MedA", "MedB", "SmallA", "SmallB","SmallC","LargeA","MiniA","MiniE"
	
	int UI_numIntervals	= 1;
	// between 1 and 5
	
	int UI_numCycles 	= 2;
	// between 1 and 500
	
	int UI_force 		= 10;
	//  force in z direction, between 10 and 80 [N]
	
	/********************************************************/
	
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
		
		F_flange = lbr_iiwa_14_R820_1.getFlange();		//flange frame
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
		
		
		
		for (int I=1; I<=5;I++ )
		{
			if(I==0)
				UI_task = "Whittling";
			else
				UI_task = "Scrapedemo";
			
			if(I==5)
			{
				UI_numCycles = 10;	
				UI_force = 40;	
			}
			
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
			case "Scrapedemo":
				myTask = new Scrapedemo(lbr_iiwa_14_R820_1, scraper, UI_force,I);
				imp = 0;
                J_standBy = J_standBy1 ;	
                break;
                    
            default:
            		System.out.println("invalid task");
            		System.exit(0); 
		}
		
		
		scraper.attachTo(F_flange);
	
		
		for (int i_inter=1; i_inter <= UI_numIntervals; i_inter++) {
			
			// go to standby position
			//System.out.println("going to standBy position");
			//scraper.move(ptp(J_standBy).setJointVelocityRel(v_taxingRel));
			if(I==1 || I==0)
			{
			System.out.println("going to standBy position");
			scraper.move(ptp(J_standBy).setJointVelocityRel(v_taxingRel));
			}
			
			
			// tell task to start with first Cycle
			myTask.firstCycle = true;
			myTask.lastCycle = false;
			
			// start cycling of one interval
			System.out.println("start cycling the task");
			
			for (int i_cycle = 1; i_cycle <= UI_numCycles; i_cycle++) {
				System.out.println(String.format("cycle # %1$d", i_cycle));
				
				if(i_cycle == UI_numCycles )
				myTask.lastCycle = true;	
				
				myTask.goCycle(imp);
				
				
				
			}
			
			// go to mounting position
			//System.out.println("Going to mounting position");	
			//MountingPosition MP = new MountingPosition();
			//MP.runApplication();
			
		}
		//if(I==0 || I==2 || I==4 || I==5)
		if(!myTask.lastCycle || I==0 || I==2 || I==4 || I==5)
		scraper.move(ptp(J_standBy).setJointVelocityRel(v_taxingRel).setJointAccelerationRel(0.1));
		//scraper.move(linRel(0,0,200).setJointVelocityRel(v_taxingRel).setJointAccelerationRel(0.1));
		//if(I==1 || I==3)
	    //scraper.move(lin().setJointVelocityRel(v_taxingRel).setJointAccelerationRel(0.1));
		}
		// go to standBy position:
		
		scraper.move(ptp(J_standBy).setJointVelocityRel(v_taxingRel).setJointAccelerationRel(0.1));
		
	}


	public static void main(String[] args) {
		KUKADemo2 app = new KUKADemo2();
		app.runApplication();
	}
}