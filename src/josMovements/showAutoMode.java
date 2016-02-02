package josMovements;

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
import com.kuka.roboticsAPI.motionModel.SplineJP;
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
public class showAutoMode extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;

	String dummy;
	
	// joint angles
	double a1,a2,a3,a4,a5,a6,a7;
		
	private static final double speed = 0.45;
	
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
	}

	public void run() {
		
		// move to vertical (home) position
		lbr_iiwa_14_R820_1.move(ptpHome().setJointVelocityRel(0.3));
		
		/*
		// facilitate inverse kinematics 
		a1 = Math.toRadians(-10);
		a2 = Math.toRadians(10);
		a3 = Math.toRadians(-10);
		a4 = Math.toRadians(-10);
		a5 = Math.toRadians(-10);
		a6 = Math.toRadians(10);
		a7 = Math.toRadians(10);
		
		lbr_iiwa_14_R820_1.move(ptp(a1,a2,a3,a4,a5,a6,a7).setJointVelocityRel(0.05));
		*/
		
		// classes needed for ForwardKinematics
		ObjectFrame flange = lbr_iiwa_14_R820_1.getFlange();
		ObjectFrame lbrRoot = lbr_iiwa_14_R820_1.getRootFrame();
		
		// show basic frame data:
		dummy = String.format("frame flange x: %1$f y: %2$f z: %3$f a: %4$f b: %5$f c: %6f", flange.getX(), flange.getY(), flange.getZ(),
				flange.getAlphaRad(), flange.getBetaRad(), flange.getGammaRad());
		System.out.println(dummy);
		dummy = String.format("frame lbrRoot x: %1$f y: %2$f z: %3$f", lbrRoot.getX(), lbrRoot.getY(), lbrRoot.getZ());
		System.out.println(dummy);
			
		// pose 1
		a1 = Math.toRadians(-90);
		a2 = Math.toRadians(20);
		a3 = Math.toRadians(-5);
		a4 = Math.toRadians(-58);
		a5 = Math.toRadians(2);
		a6 = Math.toRadians(100);
		a7 = Math.toRadians(0);
		
		JointPosition pose1_joint = new JointPosition(a1,a2,a3,a4,a5,a6,a7);
		Frame pose1_frame = new Frame(lbrRoot);
		pose1_frame.setTransformationFromParent(lbr_iiwa_14_R820_1.getForwardKinematic(pose1_joint));

		System.out.println("Frame 1");
		dummy = String.format("x: %1$f y: %2$f z: %3$f", pose1_frame.getX(), pose1_frame.getY(), pose1_frame.getZ());
		System.out.println(dummy);
		
		// pose 2
		a1 = Math.toRadians(42);
		a2 = Math.toRadians(105);
		a3 = Math.toRadians(42);
		a4 = Math.toRadians(43);
		a5 = Math.toRadians(-40);
		a6 = Math.toRadians(62);
		a7 = Math.toRadians(0);
		
		JointPosition pose2_joint = new JointPosition(a1,a2,a3,a4,a5,a6,a7);
		Frame pose2_frame = new Frame(lbrRoot);
		pose2_frame.setTransformationFromParent(lbr_iiwa_14_R820_1.getForwardKinematic(pose2_joint));

		System.out.println("Frame 2");
		dummy = String.format("x: %1$f y: %2$f z: %3$f", pose2_frame.getX(), pose2_frame.getY(), pose2_frame.getZ());
		System.out.println(dummy);
		
		// pose 3
		a1 = Math.toRadians(42);
		a2 = Math.toRadians(105);
		a3 = Math.toRadians(42);
		a4 = Math.toRadians(43);
		a5 = Math.toRadians(-40);
		a6 = Math.toRadians(62);
		a7 = Math.toRadians(0);
		
		JointPosition pose3_joint = new JointPosition(a1,a2,a3,a4,a5,a6,a7);
		Frame pose3_frame = new Frame(lbrRoot);
		pose3_frame.setTransformationFromParent(lbr_iiwa_14_R820_1.getForwardKinematic(pose3_joint));

		System.out.println("Frame 3");
		dummy = String.format("x: %1$f y: %2$f z: %3$f", pose3_frame.getX(), pose3_frame.getY(), pose3_frame.getZ());
		System.out.println(dummy);
		
		// pose 4
		a1 = Math.toRadians(0);
		a2 = Math.toRadians(103);
		a3 = Math.toRadians(0);
		a4 = Math.toRadians(100);
		a5 = Math.toRadians(0);
		a6 = Math.toRadians(-105);
		a7 = Math.toRadians(0);
				
		JointPosition pose4_joint = new JointPosition(a1,a2,a3,a4,a5,a6,a7);
		Frame pose4_frame = new Frame(lbrRoot);
		pose4_frame.setTransformationFromParent(lbr_iiwa_14_R820_1.getForwardKinematic(pose4_joint));

		System.out.println("Frame 4");
		dummy = String.format("x: %1$f y: %2$f z: %3$f", pose4_frame.getX(), pose4_frame.getY(), pose4_frame.getZ());
		System.out.println(dummy);
		
		// pose 5
		a1 = Math.toRadians(-30);
		a2 = Math.toRadians(45);
		a3 = Math.toRadians(0);
		a4 = Math.toRadians(0);
		a5 = Math.toRadians(0);
		a6 = Math.toRadians(0);
		a7 = Math.toRadians(0);
				
		JointPosition pose5_joint = new JointPosition(a1,a2,a3,a4,a5,a6,a7);
		Frame pose5_frame = new Frame(lbrRoot);
		pose5_frame.setTransformationFromParent(lbr_iiwa_14_R820_1.getForwardKinematic(pose5_joint));

		System.out.println("Frame 5");
		dummy = String.format("x: %1$f y: %2$f z: %3$f", pose5_frame.getX(), pose5_frame.getY(), pose5_frame.getZ());
		System.out.println(dummy);
		
	
		// MOVE:
		/*
		lbr_iiwa_14_R820_1.move(ptp(pose1_joint).setJointVelocityRel(speed));
		lbr_iiwa_14_R820_1.move(ptp(pose2_joint).setJointVelocityRel(speed));
		lbr_iiwa_14_R820_1.move(ptp(pose3_joint).setJointVelocityRel(speed));
		lbr_iiwa_14_R820_1.move(ptp(pose4_joint).setJointVelocityRel(speed));
		lbr_iiwa_14_R820_1.move(ptp(pose5_joint).setJointVelocityRel(speed));
		
		lbr_iiwa_14_R820_1.move(ptpHome().setJointVelocityRel(speed));
		*/
		
		// JP spline motion:
		SplineJP mySpline = new SplineJP(
				ptp(pose1_joint),
				ptp(pose2_joint),
				ptp(pose3_joint),
				ptp(pose4_joint),
				ptp(pose5_joint),
				ptpHome()
				).setJointVelocityRel(speed);
		
		lbr_iiwa_14_R820_1.move(mySpline);
		
		
	}

	
	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		showAutoMode app = new showAutoMode();
		app.runApplication();
	}
}
