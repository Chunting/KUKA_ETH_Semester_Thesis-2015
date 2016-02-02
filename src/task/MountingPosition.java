package task;

//import data.DataHandler;



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
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;



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
public class MountingPosition extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	
	// standBy position in joint space
	double a1 = Math.toRadians(0);		// joint angles
	double a2 = Math.toRadians(48);
	double a3 = Math.toRadians(0);
	double a4 = Math.toRadians(-76);
	double a5 = Math.toRadians(75);
	double a6 = Math.toRadians(86);
	double a7 = Math.toRadians(-60);
	
	JointPosition J_mounting = new JointPosition(a1,a2,a3,a4,a5,a6,a7);

	
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,"LBR_iiwa_14_R820_1");
		
	}

	
	public void run() {
	
		// define frames for task trajectory:
    Frame F_mounting = new Frame(lbr_iiwa_14_R820_1.getRootFrame());
	F_mounting.setTransformationFromParent(lbr_iiwa_14_R820_1.getForwardKinematic(J_mounting));
	lbr_iiwa_14_R820_1.move(lin(F_mounting).setJointVelocityRel(0.25));	
		
	
	}
	
	
	public static void main(String[] args) {
		MountingPosition app = new MountingPosition();
		app.runApplication();
	}	
}

