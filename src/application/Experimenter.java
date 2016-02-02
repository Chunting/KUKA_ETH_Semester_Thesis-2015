package application;


import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.net.Socket;
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
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;

import task.Itask;
import task.MountingPosition;
import task.Scraping5;
import task.ScrapeSmooth;
import task.Scrapingbone;
import task.Whittling3;
import task.Microscopy;
import tcp.FileEvent;
import task.Sawing1;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a  
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */

public class Experimenter extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	private Tool scraper;
	
	//Scraping5 myTask;
	Itask myTask;
	private int imp = 0;
	
	private static final double v_taxingRel = 0.15;		// relative velocity for non task movements [1]
	
	private ObjectFrame F_flange;		//flange frame
	private ObjectFrame F_root;			//root frame
		
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
	JointPosition J_standBy ;
	
	private int numCyclesPerInter;
	private int numInter;
	
	private double force;	// force in z direction [N]
	
	
	double A1 = Math.toRadians(40);		// joint angles
	double A2 = Math.toRadians(68);
	double A3 = Math.toRadians(0);
	double A4 = Math.toRadians(-94);
	double A5 = Math.toRadians(42);
	double A6 = Math.toRadians(102);
	double A7 = Math.toRadians(-35);
	
	int UI_focus;
	JointPosition J_micro =  new JointPosition(A1,A2,A3,A4,A5,A6,A7);
	
	
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
		
		F_flange = lbr_iiwa_14_R820_1.getFlange();		//flange frame
		F_root = lbr_iiwa_14_R820_1.getRootFrame();		//root frame
	}

	
	public void run() {
		
		int UI_task  				= getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Select task", "Scraping", "Whittling","ScrapingImp","WhittlingImp","Sawing","Scrapingbone","ScrapeSmooth");
		int UI_tool  				= getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Select tool", "MedA", "MedB", "SmallA", "SmallB","SmallC","LargeA","MiniA","MiniE");
		int UI_numCyclesPerInter  	= getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Interval size?","30", "50", "100","1","5");
		int UI_numInter				= getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "How many intervals?", "1", "2", "3", "4", "5");
		int UI_force			    = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "How much force?", "10", "20", "30","40", "50","60","70");
		
		System.out.println(String.format("integer is: %d",UI_task));
		System.out.println(String.format("integer is: %d",UI_numCyclesPerInter));
		System.out.println(String.format("integer is: %d",UI_numInter));
		System.out.println(String.format("integer is: %d",UI_tool));
		
		String t = "MedA";
		switch (UI_tool){
			case 0: scraper = getApplicationData().createFromTemplate("ScraperMediumA");
			        t = "MedA" ;
					break;
			case 1: scraper = getApplicationData().createFromTemplate("ScraperMediumB");
			        t = "MedB" ;
					break;
			case 2: scraper = getApplicationData().createFromTemplate("ScraperSmallA");
			        t = "SmallA" ;
					break;
			case 3: scraper = getApplicationData().createFromTemplate("ScraperSmallB");
			        t = "SmallB" ;
					break;
			case 4: scraper = getApplicationData().createFromTemplate("ScraperSmallC");
	                t = "SmallC" ;
			        break;
			case 5: scraper = getApplicationData().createFromTemplate("ScraperLargeA");
            		t = "LargeA" ;
            		break;			
			case 6: scraper = getApplicationData().createFromTemplate("MiniA");
    		        t = "MiniA" ;
    		        break;
			case 7: scraper = getApplicationData().createFromTemplate("MiniE");
	                t = "MiniE" ;
	                 break;
		}
		 
		
		
		switch (UI_force){
		case 0: force = 10;
				break;
		case 1: force = 20;
				break;
		case 2: force = 30;
				break;
		case 3: force = 40;
				break;
		case 4: force = 50;
				break;
		case 5: force = 60;
				break;
		case 6: force = 70;
				break;
		
	}
		int force1 = (int) force ;
        String f = 	String.format("F%d",force1);
        
        switch (UI_numCyclesPerInter){
		case 0: numCyclesPerInter = 30;
				break;
		case 1: numCyclesPerInter = 50;
				break;
		case 2: numCyclesPerInter = 100;
				break;
		case 3: numCyclesPerInter = 1;
		        break;
		case 4: numCyclesPerInter = 5;
                break;
	}
	String ncpi = 	String.format("NCPI%d",numCyclesPerInter);
	
	switch (UI_numInter){
		case 0: numInter = 1;
				break;
		case 1: numInter = 2;
				break;
		case 2: numInter = 3;
				break;
		case 3: numInter = 4;
				break;
		case 4: numInter = 5;
				break;
	}
	
        
        String task = "Scrap" ;
		switch (UI_task){
			case 0: myTask = new Scraping5(lbr_iiwa_14_R820_1, scraper, force);
			        imp = 0;
			        task = "Scrap";
			        J_standBy = J_standBy1 ;
					break;
			case 1: myTask = new Whittling3(lbr_iiwa_14_R820_1, scraper, force);
			        imp = 0;
			        task = "Whitt";
			        J_standBy = J_standBy2 ;					
			        break;
			case 2: myTask = new Scraping5(lbr_iiwa_14_R820_1, scraper, force);
			        imp = 1;
			        task = "ScrapImp";
			        J_standBy = J_standBy1 ;	
					break;
			case 3:  myTask = new Whittling3(lbr_iiwa_14_R820_1, scraper, force);
			        imp = 1;
			        task = "WhittImp";
			        J_standBy = J_standBy2 ;	
					break;
			case 4: myTask = new Sawing1(lbr_iiwa_14_R820_1, scraper, force, numCyclesPerInter);
	                //imp = 1;
			        task = "Saw";
	                J_standBy = J_standBy2 ;	
	                numCyclesPerInter = 1;
			        break;
			case 5: myTask = new Scrapingbone(lbr_iiwa_14_R820_1, scraper, force);
                    //imp = 1;
	                task = "ScrapBone";
                    J_standBy = J_standBy1 ;	
	                break;       
			case 6: myTask = new ScrapeSmooth(lbr_iiwa_14_R820_1, scraper, force);
                    //imp = 1;
                    task = "Scrapsmooth";
                    J_standBy = J_standBy1 ;	
                    break;
			
		}
		
		
		
		scraper.attachTo(F_flange);

		//Creating file name parameter
		DateFormat d = new SimpleDateFormat("dd-MM-yyyy-HH-mm-ss");
	    Date date = new Date();
	    String dat = d.format(date);

		
		
		// standby position
		System.out.println("going to standBy position");
		scraper.move(ptp(J_standBy).setJointVelocityRel(v_taxingRel));
		
		// start cycling
		System.out.println("start cycling the task");
	
		
		for (int i_inter=1; i_inter <= numInter; i_inter++) {
			
			String ni = 	String.format("NI%d",i_inter);
			
			String filename = dat + t + f + task + ncpi + ni + ".log" ; // creating the filename using the required parameters
			
			System.out.println(filename);
			
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
			
			
			
			// one interval experiment 
			for (int i_cycle = 1; i_cycle <= numCyclesPerInter; i_cycle++) {
				System.out.println(String.format("cycle # %1$d", i_cycle));
				
				myTask.goCycle(imp);
				
			}
			
			
			
			// stop recording
			rec.stopRecording();
			System.out.println("data recording stopped");	
			
			System.out.println("Going to mounting position");	
			
			// go to mounting position
			MountingPosition MP = new MountingPosition();
			MP.runApplication();
			
		   
			
		    int dir = getApplicationUI().displayModalDialog( ApplicationDialogType.QUESTION, "Do you want to get data?", "Yes", "No");
			if(dir == 0)
			filesend(filename);
			
			//go to microscope
			dir = getApplicationUI().displayModalDialog( ApplicationDialogType.QUESTION, "Do you want to go to microscope?", "Yes", "No");
			if(dir == 0)
			{
		     microscopy();
			}
			
			
			
			dir = getApplicationUI().displayModalDialog( ApplicationDialogType.QUESTION, "Do you want to continue now?", "Yes", "No");
			if(dir == 1)
		    break;
			
		}
		 
			
		// move to standBy position:
		scraper.move(ptp(J_standBy).setJointVelocityRel(v_taxingRel).setJointAccelerationRel(0.1));
		
	}
	
//********************************************************************************//	
	
	private void microscopy() {
		
		Frame F_micro = new Frame(lbr_iiwa_14_R820_1.getRootFrame());
		F_micro.setTransformationFromParent(lbr_iiwa_14_R820_1.getForwardKinematic(J_micro));
		Frame F_micro1 = F_micro.copy(); 
		F_micro1.setX(F_micro.getX()+30);
		F_micro1.setY(F_micro.getY()-30);
		F_micro1.setZ(F_micro.getZ()+25);
		
		//creating user keybar
		IUserKeyBar keybar = getApplicationUI().createUserKeyBar("keybar");
		
			
			IUserKeyListener listener0 = new IUserKeyListener() {

				//@Override
				public void onKeyEvent( IUserKey key0,UserKeyEvent event)
				{	
			     if(event == UserKeyEvent.KeyDown)
			     {
			    	 switch (UI_focus){
			 		case 0: scraper.move(linRel(0.2,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
			 				break;
			 		case 1: scraper.move(linRel(0,0.2,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
			 				break;
			 		case 2: scraper.move(linRel(0,0,0.1,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
			 				break;
			 		case 3: break;
			    	 }
			     }
			     
				}
			};
			
			IUserKeyListener listener1 = new IUserKeyListener() {

				//@Override
				public void onKeyEvent( IUserKey key1,UserKeyEvent event)
				{	
					if(event == UserKeyEvent.KeyDown)
				     {
				
					switch (UI_focus){
			 		case 0: scraper.move(linRel(-0.2,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
			 				break;
			 		case 1: scraper.move(linRel(0,-0.2,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
			 				break;
			 		case 2: scraper.move(linRel(0,0,-0.1,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
			 				break;
			 		case 3: break;
			 		default : 
			    	 }
				     }
				}
			};
			
			
			
			IUserKey key0 = keybar.addUserKey(0, listener0,true);
		    key0.setText(UserKeyAlignment.TopMiddle , "XYZ+");
		    
		    
		    IUserKey key1 = keybar.addUserKey(1, listener1,true);
		    key1.setText(UserKeyAlignment.TopMiddle , "XYZ-");
		   
		    
		    
		    
		    keybar.publish();
		    
		    
		
		    scraper.move(lin(F_micro1).setJointVelocityRel(0.25));	
	//	lbr_iiwa_14_R820_1.move(ptp(J_micro).setJointVelocityRel(0.25));
		    scraper.move(linRel(-210,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.1));
		    scraper.move(linRel(-50,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
		    scraper.move(linRel(0,0,0,0,0,Math.toRadians(-45),lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.2));
		    
	   
		while(true)
		{
			UI_focus  = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Do you want to focus", "X","Y","Z","No");
			if (UI_focus == 3) 
		    break;
		}
		
		
		getApplicationControl().pause();
		
		scraper.move(linRel(50,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
		scraper.move(linRel(100,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.1));
		
	}

//********************************************************************************************************//
	private void filesend(String filename) {
	    
	    Socket socket = null;
	    ObjectOutputStream outputStream = null;
	    boolean isConnected = false;
	    String sourceFilePath = "C:/KRC/Roboter/Log/DataRecorder/"+ filename;
	    FileEvent fileEvent = null;
	    String destinationPath_PC = "C:/Users/RoboCut/Desktop/experiments/" ;
	    System.out.println(destinationPath_PC);
	    	
		while (!isConnected) {
            try {
                socket = new Socket("172.31.1.1", 4445);
                outputStream = new ObjectOutputStream(socket.getOutputStream());
                isConnected = true;
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
		
		fileEvent = new FileEvent();
        String fileName = sourceFilePath.substring(sourceFilePath.lastIndexOf("/") + 1, sourceFilePath.length());
        String path = sourceFilePath.substring(0, sourceFilePath.lastIndexOf("/") + 1);
        fileEvent.setDestinationDirectory(destinationPath_PC);
        fileEvent.setFilename(fileName);
        fileEvent.setSourceDirectory(sourceFilePath);
        File file = new File(sourceFilePath);
        if (file.isFile()) {
            try {
               DataInputStream diStream = new DataInputStream(new FileInputStream(file));
                long len = (int) file.length();
                byte[] fileBytes = new byte[(int) len];
                int read = 0;
                int numRead = 0;
                while (read < fileBytes.length && (numRead = diStream.read(fileBytes, read,
                        fileBytes.length - read)) >= 0) {
                    read = read + numRead;
                }
                fileEvent.setFileSize(len);
                fileEvent.setFileData(fileBytes);
                fileEvent.setStatus("Success");
            } catch (Exception e) {
                e.printStackTrace();
                fileEvent.setStatus("Error");
            }
        } else {
            System.out.println("path specified is not pointing to a file");
            fileEvent.setStatus("Error");
        }
        //Now writing the FileEvent object to socket
        try {
            outputStream.writeObject(fileEvent);
            System.out.println("Done...Going to exit");
            Thread.sleep(3000);
            //System.exit(0);
        } catch (IOException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
       
		
	}
	
	
	
	
	public static void main(String[] args) {
		Experimenter app = new Experimenter();
		app.runApplication();
	}
}
