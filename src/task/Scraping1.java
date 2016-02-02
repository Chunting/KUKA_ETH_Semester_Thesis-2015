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
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
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

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;

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
public class Scraping1 extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	
	String dummy;
	
	// Forces
	final static double force_z = 20;	// force in z direction [N]
	
	// Velocities
	private static final double v_transition = 80;		// velocity during transition phase [mm/s]
	private static final double v_contact = 100;		// velocity during contact phase [mm/s]
	private static final double v_fly = 260;			// velocity during flying phase [mm/s]
	private static final double v_taxing = 160;			// velocity for non task movements [mm/s]
	private static final double vRel_taxing = 0.25;		// relative velocity for non task movements [1]
	
	private static final double dx = 140;	//moving in base x [mm]
	private static final double dz = 100;	//moving in base z [mm]
	
	// Frames
	Frame F_contactEnd;
	Frame F_contactStart;
	Frame F_flyMin;
	Frame F_flyMax;
	Frame F_fly3;
	Frame F_fly4;
	
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
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
	}

	
	
	public void run() {
		
	// get relevant robot frames
	ObjectFrame flange = lbr_iiwa_14_R820_1.getFlange();		//flange frame
	ObjectFrame lbrRoot = lbr_iiwa_14_R820_1.getRootFrame();	//root frame
			
	// define frames for task trajectory:
	Frame F_contactEnd = new Frame(lbrRoot,500,0,385,Math.toRadians(180),Math.toRadians(0),Math.toRadians(180));
	
	Frame F_tmp = F_contactEnd.copy();
	
	F_flyMin 		= F_tmp.transform(lbrRoot,XyzAbcTransformation.ofTranslation( -40,0,  50)).copy();
	F_fly3 			= F_tmp.transform(lbrRoot,XyzAbcTransformation.ofTranslation(  40,0,  60)).copy();
	F_fly4 			= F_tmp.transform(lbrRoot,XyzAbcTransformation.ofTranslation( 200,0,   0)).copy();
	F_flyMax 		= F_tmp.transform(lbrRoot,XyzAbcTransformation.ofTranslation(  40,0, -60)).copy();		
	F_contactStart 	= F_tmp.transform(lbrRoot,XyzAbcTransformation.ofTranslation( -40,0, -50)).copy();
	
	System.out.println(String.format("F_flyMin, x: %1$f y: %2$f z: %3$f", F_flyMin.getX(), F_flyMin.getY(), F_flyMin.getZ()));
	System.out.println(String.format("F_fly3, x: %1$f y: %2$f z: %3$f", F_fly3.getX(), F_fly3.getY(), F_fly3.getZ()));
	System.out.println(String.format("F_fly4, x: %1$f y: %2$f z: %3$f", F_fly4.getX(), F_fly4.getY(), F_fly4.getZ()));
	System.out.println(String.format("F_flyMax, x: %1$f y: %2$f z: %3$f", F_flyMax.getX(), F_flyMax.getY(), F_flyMax.getZ()));
	System.out.println(String.format("F_contactStart, x: %1$f y: %2$f z: %3$f", F_contactStart.getX(), F_contactStart.getY(), F_contactStart.getZ()));
	
	Spline flyingPhase_spline = new Spline(
			lin(F_contactEnd),
			spl(F_flyMin).setOrientationType(SplineOrientationType.Ignore),
			lin(F_fly3).setOrientationType(SplineOrientationType.Ignore),
			spl(F_fly4).setOrientationType(SplineOrientationType.Ignore),
			lin(F_flyMax).setOrientationType(SplineOrientationType.Ignore),
			spl(F_contactStart)
		);
	
	
	// move to standBy position:
	System.out.println("going to standBy position");
	lbr_iiwa_14_R820_1.move(ptp(J_standBy).setJointVelocityRel(vRel_taxing));
	

	
	/*
	System.out.println("going to end frame of contact phase");
	lbr_iiwa_14_R820_1.move(lin(F_contactEnd).setCartVelocity(v_taxing));
	*/
	
	/*
	lbr_iiwa_14_R820_1.move(lin(F_flyMin).setCartVelocity(v_taxing));
	lbr_iiwa_14_R820_1.move(lin(F_fly3).setCartVelocity(v_taxing));
	lbr_iiwa_14_R820_1.move(lin(F_fly4).setCartVelocity(v_taxing));
	
	lbr_iiwa_14_R820_1.move(lin(F_flyMax).setCartVelocity(v_taxing));
	lbr_iiwa_14_R820_1.move(lin(F_contactStart).setCartVelocity(v_taxing));
	lbr_iiwa_14_R820_1.move(lin(F_contactEnd).setCartVelocity(v_taxing)); 
	*/
	
	/*
	System.out.println("start flying phase - spline (3x)");
	for (int i=1; i<=3; i++)
	{
		lbr_iiwa_14_R820_1.move(flyingPhase_spline.setCartVelocity(v_fly).setCartAcceleration(250));
	}
	*/
	
	/*
	System.out.println("start flying phase - batch");
	lbr_iiwa_14_R820_1.moveAsync(batch(
			lin(F_contactEnd),
			lin(F_flyMin).setOrientationType(SplineOrientationType.Ignore),
			lin(F_fly3).setOrientationType(SplineOrientationType.Ignore),
			lin(F_fly4).setOrientationType(SplineOrientationType.Ignore),
			lin(F_flyMax).setOrientationType(SplineOrientationType.Ignore),
			lin(F_contactStart)
	).setCartVelocity(v_fly));
	*/
	
	
	System.out.println("Setting impedance mode");
	CartesianImpedanceControlMode cartImpMode = new CartesianImpedanceControlMode();

	/*
	cartImpMode.parametrize(CartDOF.TRANSL).setStiffness(1500);
	cartImpMode.parametrize(CartDOF.TRANSL).setDamping(0.7);

	cartImpMode.parametrize(CartDOF.Z).setStiffness(100);

	cartImpMode.parametrize(CartDOF.ROT).setStiffness(200);
	cartImpMode.parametrize(CartDOF.ROT).setDamping(0.7);

	cartImpMode.setNullSpaceDamping(0.3);
	cartImpMode.setNullSpaceStiffness(0);

	cartImpMode.setMaxControlForce(30, 30, 30, 10, 10, 10, false);
	*/
	
	// manage data recording
	DataRecorder rec = new DataRecorder(); //data recording 
	rec.setFileName("Recording.log");
	rec.setSampleRate(100);
	rec.addCartesianForce(lbr_iiwa_14_R820_1.getFlange(),lbrRoot);
	StartRecordingAction startAction = new StartRecordingAction(rec);
	
	ForceCondition condstart = ForceCondition.createNormalForceCondition(null, CoordinateAxis.Z, 1); //recording trigger
	ForceCondition condend = ForceCondition.createNormalForceCondition(null, CoordinateAxis.Z, force_z); //stop movement trigger
	
	
	System.out.println("going to start frame of contact phase");
	lbr_iiwa_14_R820_1.move(lin(F_contactStart).setCartVelocity(v_taxing).setCartAcceleration(200));	
	
	//start cycling
	for (int i_cycle=1; i_cycle <= 3; i_cycle ++)
	{
		System.out.println(String.format("cycle # %1$d", i_cycle));
		
		//PHASE 1: transition, landing
		//System.out.println("start transition phase, landing, impedance mode ");
		//lbr_iiwa_14_R820_1.move(linRel(0,0,-dz,lbrRoot).triggerWhen(condstart, startAction).breakWhen(condend).setCartVelocity(v_transition).setMode(cartImpMode));
		
		if (i_cycle == 1)
		{
			// check data recording
			System.out.println("recording activated");
			if(rec.isFileAvailable())
				System.out.println("data file available");
			
			URL url = rec.getURL();
			dummy = url.toString();
			System.out.println(dummy);
		}
		
		// print current force value
		ForceSensorData data = lbr_iiwa_14_R820_1.getExternalForceTorque(lbr_iiwa_14_R820_1.getFlange());
		System.out.println(String.format("Fz = %1$f ",data.getForce().getZ()));
		
		// PHASE 2: contact phase
		//System.out.println("start contact phase, impedance mode");
		//lbr_iiwa_14_R820_1.move(lin(F_contactEnd).setCartVelocity(v_contact).setMode(cartImpMode));
		/*
		// PHASE 3: transition phase, take off
		System.out.println("start transition phase, impedance mode");
		lbr_iiwa_14_R820_1.move(linRel(0,0,dz,lbrRoot).setCartVelocity(v_transition).setMode(cartImpMode));
		*/
		// PHASE 4: flying phase 
		System.out.println("start flying phase, swift");
		lbr_iiwa_14_R820_1.move(flyingPhase_spline.setCartVelocity(v_fly).setCartAcceleration(250).setMode(cartImpMode));
	}
	
	
	// stop recording
	rec.stopRecording();
	System.out.println("data recording stopped");
	
	
	// move to standBy position:
	lbr_iiwa_14_R820_1.move(ptp(J_standBy).setJointVelocityRel(vRel_taxing));
		
	}
	
}
