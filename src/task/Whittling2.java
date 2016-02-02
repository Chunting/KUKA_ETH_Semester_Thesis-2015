package task;

//import data.DataHandler;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.net.Socket;
import java.net.URL;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import tcp.FileEvent;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPITask;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
//import static com.kuka.roboticsAPI.motionModel.MMCMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
//import com.kuka.roboticsAPI.geometricModel.CartDOF;
//import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;

import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
//import com.kuka.roboticsAPI.geometricModel.Tool;
//import com.kuka.roboticsAPI.motionModel.BasicMotions;
//import com.kuka.roboticsAPI.motionModel.IMotionContainer;
//import com.kuka.roboticsAPI.motionModel.SplineJP;
//import com.kuka.roboticsAPI.motionModel.CartesianPTP;
//import com.kuka.roboticsAPI.deviceModel.VersionInfoGroup;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.StartRecordingAction;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;


//import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

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
public class Whittling2 extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	private Tool Scraper;
	String dummy;
	
	// Forces
	final static double force_z = 25;	// force in z direction [N]
	
	// Velocities
	private static final double v_transition = 10;		// velocity during transition phase [mm/s]
	private static final double v_transition_impact = 40 ; // velocity during transition phase for impact[mm/s]
	private static final double v_contact = 100;		// velocity during contact phase [mm/s]
	private static final double v_fly = 240;			// velocity during flying phase [mm/s]		
	private static final double v_taxing = 140;			// velocity for non task movements [mm/s]
	private static final double vRel_taxing = 0.15;		// relative velocity for non task movements [1]
	
	private static final double transition_dz = 50;		//moving in base z [mm]
	private static final double contact_dx = 240;
	private static final double refMid_dz = 0;
	private static final double x_start = 570;
	private static final double workingangle = 55;
	
	// Frames
	Frame F_contactEnd;
	Frame F_contactStart;
	Frame F_refStart;
	Frame F_refMid;
	Frame F_flightmid;
	Frame F_contactEndup;

	
	// standBy position in joint space
	double a1 = Math.toRadians(0);		// joint angles
	double a2 = Math.toRadians(45);
	double a3 = Math.toRadians(0);
	double a4 = Math.toRadians(-45);
	double a5 = Math.toRadians(0);
	double a6 = Math.toRadians(-20);
	double a7 = Math.toRadians(0);
	
	JointPosition J_standBy = new JointPosition(a1,a2,a3,a4,a5,a6,a7);
	
	ForceSensorData data;
	int k=0;
	
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,"LBR_iiwa_14_R820_1");
		Scraper = getApplicationData().createFromTemplate("ScraperSmallC");
	}

	
	public void run() {
		
		int direction = getApplicationUI().displayModalDialog( ApplicationDialogType.QUESTION, "What do you want to do ?", "Whittle", "Get data", "BE AWESOME" );
		
		switch (direction)
		{
		case 0:
		
		// get relevant robot frames
		ObjectFrame flange = lbr_iiwa_14_R820_1.getFlange();		//flange frame
		ObjectFrame lbrRoot = lbr_iiwa_14_R820_1.getRootFrame();	//root frame
		
		// connect tool to flange
		Scraper.attachTo(flange);
		//Scraper.setDefaultMotionFrame(Scraper.getFrame("/ToolTip"));
		//if (Scraper.getDefaultMotionFrame().getName() =! "ToolTip") error
		
		ObjectFrame ToolTip = Scraper.getFrame("/ToolTip");
		
		
		// define frames for task trajectory:
		Frame F_standBy = new Frame(lbrRoot);
		F_standBy.setTransformationFromParent(lbr_iiwa_14_R820_1.getForwardKinematic(J_standBy));
		
		//F_contactStart 	= new Frame(lbrRoot,710,-10,100,Math.toRadians(-145.8),Math.toRadians(9),Math.toRadians(-174));
		//F_contactEnd 	= new Frame(lbrRoot,480,-10,120,Math.toRadians(-146.3),Math.toRadians(1),Math.toRadians(-179.6));
		
		F_contactStart 	= new Frame(lbrRoot,x_start					,0, 80,Math.toRadians(-180),Math.toRadians(workingangle),Math.toRadians(180));
		F_contactEnd 	= new Frame(lbrRoot,x_start+contact_dx		,0, 70,Math.toRadians(-180),Math.toRadians(workingangle),Math.toRadians(180));
		F_contactEndup  = new Frame(lbrRoot,x_start+contact_dx		,0, 80,Math.toRadians(-180),Math.toRadians(workingangle),Math.toRadians(180));
		F_flightmid     = new Frame(lbrRoot,x_start+(0.5*contact_dx),0,100,Math.toRadians(-180),Math.toRadians(workingangle+10),Math.toRadians(180));
		
		F_refStart 	= F_contactStart.copy();
		F_refMid 	= F_contactStart.copy();
		F_refMid.transform(lbrRoot,XyzAbcTransformation.ofTranslation(+0.9*contact_dx,0,0));	//translation relative to lbrRoot
		
		
		//LBRE1Redundancy FrameRedundancy = new LBRE1Redundancy(0,2,01001100);
		//F_contactStart.setRedundancyInformation(lbr_iiwa_14_R820_1, FrameRedundancy);
		//System.out.println(String.format("F_contactStart Redundancy, E1: %1$f, Status: %2$d Turn: %3$d",FrameRedundancy.getE1(), FrameRedundancy.getStatus(), FrameRedundancy.getTurn()));
		

		
		// move to standBy position:
		System.out.println("going to standBy position");
		Scraper.move(ptp(J_standBy).setJointVelocityRel(vRel_taxing));
		
		// set-up the impedance mode
		System.out.println("Setting impedance mode");
		CartesianImpedanceControlMode cartImpMode = new CartesianImpedanceControlMode();
			
		cartImpMode.parametrize(CartDOF.X,CartDOF.Y).setStiffness(4000);  // max 5000
		cartImpMode.parametrize(CartDOF.Z).setStiffness(2500);
		cartImpMode.parametrize(CartDOF.TRANSL).setDamping(0.8);
	
		cartImpMode.parametrize(CartDOF.ROT).setStiffness(280);		// max 300
		cartImpMode.parametrize(CartDOF.ROT).setDamping(0.9);
	 
		cartImpMode.setNullSpaceStiffness(1000);
	
		cartImpMode.setMaxControlForce(70, 70, 70, 10, 10, 10, true);
		
		//creating user keybar
				IUserKeyBar keybar = getApplicationUI().createUserKeyBar("keybar");
				
					
					IUserKeyListener listener = new IUserKeyListener() {

						//@Override
						public void onKeyEvent( IUserKey key,UserKeyEvent event)
						{	
					     if(event == UserKeyEvent.KeyDown)
						 k = 1;
						}
					};
					
					IUserKey key = keybar.addUserKey(0, listener,true);
				    key.setText(UserKeyAlignment.TopMiddle , "data");
				    keybar.publish();
		
		// manage data recording
		DataRecorder rec = new DataRecorder(); //data recording 
		rec.setFileName("Recording.log");
		rec.setSampleRate(100);
		rec.addCartesianForce(ToolTip, lbrRoot);
		rec.addCurrentCartesianPositionXYZ(ToolTip, lbrRoot);
		
		ForceCondition condend = ForceCondition.createNormalForceCondition(ToolTip,lbrRoot, CoordinateAxis.Z, force_z); //stop movement trigger
		
		// start and check data recording
		rec.enable();
		rec.startRecording();
		System.out.println("recording activated");
		if(rec.isFileAvailable())
			System.out.println("data file available");
		System.out.println(rec.getURL().toString());
		
		
		// start movement
		System.out.println("going to start frame of contact phase");
		Scraper.move(lin(F_contactStart).setCartVelocity(v_taxing).setCartAcceleration(200));
		
		int imp = getApplicationUI().displayModalDialog( ApplicationDialogType.QUESTION, "Do you want impact ?", "Yes", "No");

		
		for (int i_cycle=1; i_cycle <= 500; i_cycle ++)
		{
			System.out.println(String.format("cycle # %1$d", i_cycle));
			
			// PHASE 1: transition, landing
			System.out.println("start landing, impedance mode ");
			
			System.out.println(String.format("position in Z before landing: %f", lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolTip, lbrRoot).getZ()));
			System.out.println(String.format("Fz = %1$f ",lbr_iiwa_14_R820_1.getExternalForceTorque(ToolTip,lbrRoot).getForce().getZ()));
			
			if (imp == 0)
			Scraper.move(linRel(0,0,-transition_dz,lbrRoot).breakWhen(condend).setCartVelocity(v_transition_impact).setMode(cartImpMode));
			else
			Scraper.move(linRel(0,0,-transition_dz,lbrRoot).breakWhen(condend).setCartVelocity(v_transition).setMode(cartImpMode));
			
			System.out.println(String.format("position in Z after landing: %f", lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolTip, lbrRoot).getZ()));
			System.out.println(String.format("Fz = %1$f ",lbr_iiwa_14_R820_1.getExternalForceTorque(ToolTip,lbrRoot).getForce().getZ()));
			
			
			// build trajectory of contact phase
			//F_refStart.setZ(lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolTip, lbrRoot).getZ());
			F_refMid.setZ(lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolTip,lbrRoot).getZ()-refMid_dz);
			System.out.println(String.format("F_refMid x: %1$f y: %2$f z: %3$f", F_refMid.getX(), F_refMid.getY(), F_refMid.getZ()));
			
			Spline cycle = new Spline (
					lin(F_refMid).setCartVelocity(v_contact),	// must be kept lin, otherwise landing is splined too !!!
					spl(F_contactEnd).setCartVelocity(v_contact).setCartAcceleration(100),
					spl(F_contactEndup).setCartVelocity(v_fly).setCartAcceleration(100),
					spl(F_flightmid).setCartVelocity(v_fly),
					spl(F_contactStart).setCartVelocity(v_fly).setCartAcceleration(100)
			).setOrientationType(SplineOrientationType.Variable);
			
			// PHASE 2 and 3: contact and flight
			System.out.println("start contact phase, impedance mode");
			Scraper.move(cycle.setMode(cartImpMode));
			
			//try {
				
			    if(! F_refStart.isCloseTo(lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolTip, lbrRoot), 2, 0.1))
			    {
			    	//getApplicationControl().pause();
			    	System.out.println("waiting for TCP");
			    }
			//} catch ( java.lang.InterruptedException ie) {
			//    System.out.println(ie);
			//}
			    if((i_cycle%200 == 0)|| k==1)
				{
			    	
					 int dir = getApplicationUI().displayModalDialog( ApplicationDialogType.QUESTION, "Do you want to get data now?", "Yes", "No");
					 if (dir == 0)
					 {rec.stopRecording();
					 System.out.println("data recording stopped");
					 }
					 filesend(dir,i_cycle,imp);
					 k=0;
				}
			    
			    
			    
		}
		
		// move to standBy position:
		Scraper.move(ptp(J_standBy).setJointVelocityRel(vRel_taxing));
		
		// stop recording
		rec.stopRecording();
		System.out.println("data recording stopped");
		
		case 1:
		    
		filesend(0,0,1);
				
	    case 2 :
		System.out.println("You are awesome. Now get back to work :P ");
		}
	}
	
	private void filesend(int dir,int i_cycle, int imp) {
	    
		   
		switch(dir)
	    {
	    case 0:
	    
	    DateFormat d = new SimpleDateFormat("dd-MM-yyyy HH-mm-ss");
	    Date date = new Date();
	    String dat = d.format(date);
	    String cycle = String.format("#%d", i_cycle);
	    String impact = "NOimp";
	    if (imp == 0)
	    impact = "imp";
	    Socket socket = null;
	    ObjectOutputStream outputStream = null;
	    boolean isConnected = false;
	    String sourceFilePath = "C:/KRC/Roboter/Log/DataRecorder/Recording.log";
	    FileEvent fileEvent = null;
	    String destinationPath_PC = "C:/Users/RoboCut/Desktop/experiments/whittle" + dat + cycle + impact ;
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
        break;
	    case 1 :
	    break;	
	    }
		
	}
	
	
	public static void main(String[] args) {
		Whittling2 app = new Whittling2();
		app.runApplication();
	}	
}

