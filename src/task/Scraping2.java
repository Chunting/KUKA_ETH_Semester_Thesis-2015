package task;

//import data.DataHandler;

import java.net.URL;

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
import com.kuka.roboticsAPI.geometricModel.math.Vector;
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
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;

import josMovements.FirstMove;

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
public class Scraping2 extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	private Tool Scraper;
	String dummy;
	
	// Forces
	final static double force_z = 20;	// force in z direction [N]
	
	// Velocities
	private static final double v_transition = 20;		// velocity during transition phase [mm/s]
	private static final double v_contact = 120;		// velocity during contact phase [mm/s]
	private static final double v_fly = 250;			// velocity during flying phase [mm/s]		
	private static final double v_taxing = 140;			// velocity for non task movements [mm/s]
	private static final double vRel_taxing = 0.15;		// relative velocity for non task movements [1]
	
	private static final double transition_dz = 50;		//moving in base z [mm]
	private static final double contact_dx = 180;
	private static final double refMid_dz = 15;
	
	// Frames
	Frame F_contactEnd;
	Frame F_contactStart;
	Frame F_refMid;
	Frame F_flightmid;
	Frame F_contactEndup;

	
	// standBy position in joint space
	double a1 = Math.toRadians(0);		// joint angles
	double a2 = Math.toRadians(45);
	double a3 = Math.toRadians(0);
	double a4 = Math.toRadians(-45);
	double a5 = Math.toRadians(0);
	double a6 = Math.toRadians(60);
	double a7 = Math.toRadians(0);
	
	JointPosition J_standBy = new JointPosition(a1,a2,a3,a4,a5,a6,a7);
	
	
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,"LBR_iiwa_14_R820_1");
		Scraper = getApplicationData().createFromTemplate("ScraperMediumA");
	}

	
	public void run() {
		
		// get relevant robot frames
		ObjectFrame flange = lbr_iiwa_14_R820_1.getFlange();		//flange frame
		ObjectFrame lbrRoot = lbr_iiwa_14_R820_1.getRootFrame();	//root frame
		
		// connect tool to flange
		Scraper.attachTo(flange);
		//Scraper.setDefaultMotionFrame(Scraper.getFrame("/ToolTip"));
		//if (Scraper.getDefaultMotionFrame().getName() =! "ToolTip") error
		
		ObjectFrame ToolTip = Scraper.getFrame("/ToolTip");
		
		ForceSensorData data;
		
		// define frames for task trajectory:
		Frame F_standBy = new Frame(lbrRoot);
		F_standBy.setTransformationFromParent(lbr_iiwa_14_R820_1.getForwardKinematic(J_standBy));
		
		//F_contactStart 	= new Frame(lbrRoot,710,-10,100,Math.toRadians(-145.8),Math.toRadians(9),Math.toRadians(-174));
		//F_contactEnd 	= new Frame(lbrRoot,480,-10,120,Math.toRadians(-146.3),Math.toRadians(1),Math.toRadians(-179.6));
		
		F_contactStart 	= new Frame(lbrRoot,870				,0,80,Math.toRadians(180),Math.toRadians(24),Math.toRadians(180));
		F_contactEnd 	= new Frame(lbrRoot,870-contact_dx	,0,70,Math.toRadians(180),Math.toRadians(24),Math.toRadians(180));
		F_contactEndup  = new Frame(lbrRoot,870-contact_dx	,0,90,Math.toRadians(180),Math.toRadians(24),Math.toRadians(180));
		F_flightmid     = new Frame(lbrRoot,870-(contact_dx*0.5) ,0,100,Math.toRadians(180),Math.toRadians(24),Math.toRadians(180));
		
		F_refMid 	= F_contactStart.copy();
		F_refMid.transform(lbrRoot,XyzAbcTransformation.ofTranslation(-0.9*contact_dx,0,0));	//translation relative to lbrRoot
		
		LBRE1Redundancy FrameRedundancy = new LBRE1Redundancy(0,2,01001100);
		F_contactStart.setRedundancyInformation(lbr_iiwa_14_R820_1, FrameRedundancy);
		System.out.println(String.format("F_contactStart Redundancy, E1: %1$f, Status: %2$d Turn: %3$d",FrameRedundancy.getE1(), FrameRedundancy.getStatus(), FrameRedundancy.getTurn()));
		

		
		// move to standBy position:
		System.out.println("going to standBy position");
		Scraper.move(ptp(J_standBy).setJointVelocityRel(vRel_taxing));
		
		// set-up the impedance mode
		System.out.println("Setting impedance mode");
		CartesianImpedanceControlMode cartImpMode = new CartesianImpedanceControlMode();
			
		cartImpMode.parametrize(CartDOF.X,CartDOF.Y).setStiffness(4000);
		cartImpMode.parametrize(CartDOF.Z).setStiffness(2000);
		cartImpMode.parametrize(CartDOF.TRANSL).setDamping(0.8);
	
		cartImpMode.parametrize(CartDOF.ROT).setStiffness(280);
		cartImpMode.parametrize(CartDOF.ROT).setDamping(0.8);
	 
		cartImpMode.setNullSpaceStiffness(1000);
	
		cartImpMode.setMaxControlForce(70, 70, 70, 10, 10, 10, true);
		
		// manage data recording
		DataRecorder rec = new DataRecorder(); //data recording 
		rec.setFileName("Recording.log");
		rec.setSampleRate(100);
		rec.addCartesianForce(flange,lbrRoot);
		rec.addCurrentCartesianPositionXYZ(flange, lbrRoot);
		
		ForceCondition condend = ForceCondition.createNormalForceCondition(ToolTip,lbrRoot, CoordinateAxis.Z, force_z); //stop movement trigger
		
		// start movement
		System.out.println("going to start frame of contact phase");
		Scraper.move(lin(F_contactStart).setCartVelocity(v_taxing).setCartAcceleration(200));
		
		// start and check data recording
		rec.enable();
		rec.startRecording();
		
		System.out.println("recording activated");
		if(rec.isFileAvailable())
			System.out.println("data file available");
		
		URL url = rec.getURL();
		dummy = url.toString();
		System.out.println(dummy);
		
		
		for (int i_cycle=1; i_cycle <= 460; i_cycle ++)
		{
			System.out.println(String.format("cycle # %1$d", i_cycle));
			
			// PHASE 1: transition, landing
			System.out.println("start transition landing, impedance mode ");
			System.out.println(String.format("position in Z before landing: %f", lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolTip, lbrRoot).getZ()));
			Scraper.move(linRel(0,0,-transition_dz,lbrRoot).breakWhen(condend).setCartVelocity(v_transition).setMode(cartImpMode));
			
			// build trajectory of contact phase
			data = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolTip,lbrRoot);
			System.out.println(String.format("Fz = %1$f ",data.getForce().getZ()));
			
			//print current position
			System.out.println(String.format("reference position after transition landing Z = %1$f ",
					lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolTip, lbrRoot).getZ()));
			
			// build trajectory of the reference point during contact phase
			F_refMid.setZ(lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolTip,lbrRoot).getZ()-refMid_dz);
			System.out.println(String.format("F_refMid x: %1$f y: %2$f z: %3$f", F_refMid.getX(), F_refMid.getY(), F_refMid.getZ()));
			
			Spline cycle = new Spline (
					lin(F_refMid).setCartVelocity(v_contact),	// must be kept lin, otherwise landing is splined too !!!
					spl(F_contactEnd).setCartVelocity(v_contact).setCartAcceleration(100),
					spl(F_contactEndup).setCartVelocity(v_fly).setCartAcceleration(100),
					spl(F_flightmid).setCartVelocity(v_fly),
					spl(F_contactStart).setCartVelocity(v_fly).setCartAcceleration(100)
			).setOrientationType(SplineOrientationType.Variable);
			
			/*
			Scraper.move(lin(F_refMid).setMode(cartImpMode).setCartVelocity(v_contact));
			Scraper.move(lin(F_contactEnd).setMode(cartImpMode).setCartVelocity(v_contact));
			Scraper.move(lin(F_contactStart).setMode(cartImpMode).setCartVelocity(v_contact));
			*/
			
			// PHASE 2 and 3: contact and flight
			System.out.println("start contact phase, impedance mode");
			Scraper.move(cycle.setMode(cartImpMode));
			
		}
		
		// stop recording
		rec.stopRecording();
		System.out.println("data recording stopped");
		
		// move to standBy position:
		Scraper.move(lin(F_standBy).setCartVelocity(v_taxing).setCartAcceleration(200));
	}
	
	
	
	public static void main(String[] args) {
		FirstMove app = new FirstMove();
		app.runApplication();
	}	
}

