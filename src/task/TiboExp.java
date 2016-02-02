package task;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPITask;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
//import static com.kuka.roboticsAPI.motionModel.MMCMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
//import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
//import com.kuka.roboticsAPI.geometricModel.Tool;
//import com.kuka.roboticsAPI.motionModel.BasicMotions;
//import com.kuka.roboticsAPI.motionModel.IMotionContainer;
//import com.kuka.roboticsAPI.motionModel.SplineJP;
//import com.kuka.roboticsAPI.motionModel.CartesianPTP;
//import com.kuka.roboticsAPI.deviceModel.VersionInfoGroup;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.StartRecordingAction;

//import java.io.BufferedInputStream;
import java.io.BufferedReader;
//import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
//import java.io.OutputStream;
//import java.net.ServerSocket;
//import java.net.Socket;
import java.net.URL;

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
public class TiboExp extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	double a1,a2,a3,a4,a5,a6,a7;
	
	String dummy;
	public static final String DataPath = "data";
	
	// parameters of the task trajectory
	private static final double dx = 180;	//moving in base x [mm]
	private static final double dz = 60;	//moving in base z [mm]
	private static final double v_transition = 0.1;	// rel. velocity during transition phase [1]
	private static final double v_contact = 0.2;		// rel. velocity during contact phase [1]
	private static final double v_fly = 0.4;			// rel. velocity during flying phase [1]
	private static final double v_journey = 0.3;		// rel. velocity for non task movements [1]
	
	private static final double force_z = 40;			// Fz in base [N]

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
	}
	
	/**
	 * Loads a frame from the frames folder of the configured data path.
	 * 
	 * @param filename
	 *            name of the frame to be loaded
	 * @return Frame object. Returns null, if the corresponding file does not exist
	 */
	public static Frame loadFrame(String filename) {
		String filePath = DataPath + "/frames/" + filename + ".frm";

		System.out.println("Reading from file '" + filePath + "'");
		Frame result = null;

		BufferedReader reader = null;
		try {
			reader = new BufferedReader(new InputStreamReader(new FileInputStream(filePath), "utf-8"));

			System.out.println("file " + filename + " is open");
			
			String[] valuesString = reader.readLine().split(" ");
			double[] valuesDouble = new double[valuesString.length];

			for (int i = 0; i < valuesString.length; i++) {
				valuesDouble[i] = Double.parseDouble(valuesString[i]);
			}

			result = new Frame(valuesDouble[0], valuesDouble[1], valuesDouble[2], Math.toRadians(valuesDouble[3]),
					Math.toRadians(valuesDouble[4]), Math.toRadians(valuesDouble[5]));

		} catch (IOException e) {
			System.err.println("could not read data from " + filePath);
		} finally {
			try {
				reader.close();
			} catch (Exception e) {
			}
		}

		return result;
	}
	

	public void run() {
		
		// show some frame data:
		ObjectFrame flange = lbr_iiwa_14_R820_1.getFlange();		//flange frame
		ObjectFrame lbrRoot = lbr_iiwa_14_R820_1.getRootFrame();	//root frame
		
		dummy = String.format("frame flange x: %1$f y: %2$f z: %3$f a: %4$f b: %5$f c: %6f", flange.getX(), flange.getY(), flange.getZ(),
				flange.getAlphaRad(), flange.getBetaRad(), flange.getGammaRad());
		System.out.println(dummy);
		dummy = String.format("frame lbrRoot x: %1$f y: %2$f z: %3$f", lbrRoot.getX(), lbrRoot.getY(), lbrRoot.getZ());
		System.out.println(dummy);
				
		// define frames:
		//Frame F_ready = new Frame(700,0,325,Math.toRadians(-180),Math.toRadians(0),Math.toRadians(180));
		Frame F_start = new Frame(685,0,208,Math.toRadians(-144),Math.toRadians(15.5),Math.toRadians(-166));
		
		a1 = Math.toRadians(0);		// joint angles for ready position
		a2 = Math.toRadians(45);
		a3 = Math.toRadians(0);
		a4 = Math.toRadians(-45);
		a5 = Math.toRadians(0);
		a6 = Math.toRadians(90);
		a7 = Math.toRadians(0);
						
		// move to ready position:
		lbr_iiwa_14_R820_1.move(ptp(a1,a2,a3,a4,a5,a6,a7).setJointVelocityRel(v_journey));

		/*
		System.out.println("move to frame F0");
		lbr_iiwa_14_R820_1.move(lin(F0).setJointVelocityRel(0.2));
		System.out.println("move to frame F1");
		lbr_iiwa_14_R820_1.move(lin(F1).setJointVelocityRel(0.2));
		 */
		
		/*
		// move to user defined position:
		double a1,a2,a3,a4,a5,a6,a7;
		a1 = Math.toRadians(0);
		a2 = Math.toRadians(45);
		a3 = Math.toRadians(0);
		a4 = Math.toRadians(-45);
		a5 = Math.toRadians(0);
		a6 = Math.toRadians(90);
		a7 = Math.toRadians(0);
		System.out.println("first motion");
		
		lbr_iiwa_14_R820_1.move(ptp(a1,a2,a3,a4,a5,a6,a7).setJointVelocityRel(0.3));
		*/
			
		/*
		Frame F1 = DataHandler.loadFrame("F1");
		Frame F2 = DataHandler.loadFrame("F2");
		Frame F3 = DataHandler.loadFrame("F3");
		Frame F4 = DataHandler.loadFrame("F4");
		*/
		
		//Tool virtualTool;
		//virtualTool.Tool("myVirtualTool");
		
		/*	
		// corner frames:
		Frame F1 = new Frame(600,-285,940,Math.toRadians(-90),Math.toRadians(0),Math.toRadians(-45));
		Frame F2 = new Frame(600,285,940,Math.toRadians(-90),Math.toRadians(0),Math.toRadians(-45));
		Frame F3 = new Frame(600,285,640,Math.toRadians(-90),Math.toRadians(0),Math.toRadians(-45));
		Frame F4 = new Frame(600,-285,640,Math.toRadians(-90),Math.toRadians(0),Math.toRadians(-45));
		
		// auxiliarry points:
		Frame Faux12 = new Frame(600,0,840,Math.toRadians(-90),Math.toRadians(0),Math.toRadians(-45));
		Frame Faux23 = new Frame(600,350,740,Math.toRadians(-90),Math.toRadians(0),Math.toRadians(-45));
		Frame Faux34 = new Frame(600,0,540,Math.toRadians(-90),Math.toRadians(0),Math.toRadians(-45));
		Frame Faux41 = new Frame(600,-350,740,Math.toRadians(-90),Math.toRadians(0),Math.toRadians(-45));
		
		CartesianPTP myCptp1 = new CartesianPTP(F1);
		CartesianPTP myCptp2 = new CartesianPTP(F2);
	//	CartesianPTP myCptp3 = new CartesianPTP(F3);
		
		//draw square with fastest (ptp) motion:
		System.out.println("draw square ptp");
		System.out.println("move to frame 1");
		lbr_iiwa_14_R820_1.move(myCptp1);
		System.out.println("move to frame 2");
		lbr_iiwa_14_R820_1.move(myCptp2);
		System.out.println("move to frame 3");
		lbr_iiwa_14_R820_1.move(ptp(F3));
		System.out.println("move to frame 4");
		lbr_iiwa_14_R820_1.move(ptp(F4));
		System.out.println("move to frame 1");
		lbr_iiwa_14_R820_1.move(ptp(F1));
		
		// draw square with linear motion:
		System.out.println("draw square linear");
		lbr_iiwa_14_R820_1.move(lin(F1).setJointVelocityRel(0.25));
		lbr_iiwa_14_R820_1.move(lin(F2).setJointVelocityRel(0.25));
		lbr_iiwa_14_R820_1.move(lin(F3).setJointVelocityRel(0.25));
		lbr_iiwa_14_R820_1.move(lin(F4).setJointVelocityRel(0.25));
		lbr_iiwa_14_R820_1.move(lin(F1).setJointVelocityRel(0.25));
	/*	
		// draw square with circular motion:
		System.out.println("draw square cicular");
		lbr_iiwa_14_R820_1.move(lin(F1));
		lbr_iiwa_14_R820_1.move(circ(Faux12,F2).setJointVelocityRel(0.25));
		lbr_iiwa_14_R820_1.move(circ(Faux23,F3).setJointVelocityRel(0.25));
		lbr_iiwa_14_R820_1.move(circ(Faux34,F4).setJointVelocityRel(0.25));
		lbr_iiwa_14_R820_1.move(circ(Faux41,F1).setJointVelocityRel(0.25));
		
	/*	
		// linear relative motion:
		lbr_iiwa_14_R820_1.move(lin(F1));
		lbr_iiwa_14_R820_1.move(linRel(0,0,-200).setJointVelocityRel(0.3));
		lbr_iiwa_14_R820_1.move(linRel(0,0,200).setJointVelocityRel(0.3));
		
		// JP spline motion:
		SplineJP mySpline = new SplineJP(
				ptp(F1),
				ptp(F2),
				ptp(F3),
				ptp(F4),
				ptp(F1)
				).setJointVelocityRel(0.75);
		
		lbr_iiwa_14_R820_1.move(mySpline);
		*/
		// go home again
	//	lbr_iiwa_14_R820_1.move(ptpHome());
		
		
		
		/*
		// move to user defined position:
		a1 = Math.toRadians(30);
		a2 = Math.toRadians(30);
		a3 = Math.toRadians(0);
		a4 = Math.toRadians(-20);
		a5 = Math.toRadians(0);
		a6 = Math.toRadians(-40);
		a7 = Math.toRadians(0);
		
		lbr_iiwa_14_R820_1.setESMState("1");
				
		lbr_iiwa_14_R820_1.move(lin(F2).setCartVelocity(300));
		lbr_iiwa_14_R820_1.move(ptpHome());
		
		//lbr_iiwa_14_R820_1.setESMState("2");
		
		lbr_iiwa_14_R820_1.move(lin(F2).setCartVelocity(300));
		lbr_iiwa_14_R820_1.move(ptpHome()); 
		
		
		lbr_iiwa_14_R820_1.move(BasicMotions.lin(F2).setCartVelocity(100));		
		lbr_iiwa_14_R820_1.move(ptpHome());
		
		lbr_iiwa_14_R820_1.setESMState("2");
		lbr_iiwa_14_R820_1.move(BasicMotions.lin(F2).setCartVelocity(100));
		
		
		lbr_iiwa_14_R820_1.move(handGuiding());
		*/
		
	//	Frame F_imp0 = new Frame(600,285,940,Math.toRadians(-90),Math.toRadians(0),Math.toRadians(-45));
	//	Frame F_imp1 = new Frame(600,285,640,Math.toRadians(-90),Math.toRadians(0),Math.toRadians(-45));
		//Frame F_imp0 = new Frame(600,0,840,Math.toRadians(-90),Math.toRadians(0),Math.toRadians(-45));
		//Frame F_imp1 = new Frame(600,0,540,Math.toRadians(-90),Math.toRadians(0),Math.toRadians(-45));

		//lbr_iiwa_14_R820_1.move(ptp(F_imp0).setJointVelocityRel(0.1));//.setJointVelocityRel(0.1));
		
	
		System.out.println("going to start position");
		lbr_iiwa_14_R820_1.move(lin(F_start).setJointVelocityRel(0.25));//tool.moveAsync(BasicMotions.lin(currentFrame).setMode(cartImpMode));
		
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
		
		
		//start cycling
		for (int i_cycle=1; i_cycle <= 3; i_cycle ++)
		{
			System.out.println(String.format("cycle # %1$d", i_cycle));
			
			//PHASE 1: transition, landing
			System.out.println("start transition phase, landing, impedance mode ");
			lbr_iiwa_14_R820_1.move(linRel(0,0,-dz,lbrRoot).triggerWhen(condstart, startAction).breakWhen(condend).setJointVelocityRel(v_transition).setMode(cartImpMode));//tool.moveAsync(BasicMotions.lin(currentFrame).setMode(cartImpMode));
			
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
			Vector force = data.getForce();
			double forceInZ = force.getZ();
			dummy = String.format("Fz = %1$f ",forceInZ );
			System.out.println(dummy);
			
			// PHASE 2: contact phase
			System.out.println("start contact phase, impedance mode");
			lbr_iiwa_14_R820_1.move(linRel(-dx,0,0,lbrRoot).setJointVelocityRel(v_contact).setMode(cartImpMode));
			
			// PHASE 3: transition phase, take off
			System.out.println("start transition phase, impedance mode");
			lbr_iiwa_14_R820_1.move(linRel(0,0,dz,lbrRoot).setJointVelocityRel(v_transition).setMode(cartImpMode));
			
			// PHASE 4: flying phase 
			System.out.println("start flying phase, swift");
			lbr_iiwa_14_R820_1.move(lin(F_start).setJointVelocityRel(v_fly));
		}
		
			
		// stop recording
		rec.stopRecording();
		System.out.println("data recording stopped");
		
		
		// move to ready position:
		lbr_iiwa_14_R820_1.move(ptp(a1,a2,a3,a4,a5,a6,a7).setJointVelocityRel(v_journey));
		
		
		/*
		try {
		ServerSocket servsock = new ServerSocket(12345);
	    File myFile = new File(dummy);
	    
	    Socket sock = servsock.accept();
	    byte[] mybytearray = new byte[(int) myFile.length()];
	    BufferedInputStream bis = new BufferedInputStream(new FileInputStream(myFile));
	    bis.read(mybytearray, 0, mybytearray.length);
	    OutputStream os = sock.getOutputStream();
	    os.write(mybytearray, 0, mybytearray.length);
	    os.flush();
	    sock.close();
	    } 
	    catch(IOException E ){
			System.out.println("Connection error"); 	
	    }
		
		*/
		
		
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		TiboExp app = new TiboExp();
		app.runApplication();
	}
}
