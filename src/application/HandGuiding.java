package application;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import static com.kuka.roboticsAPI.motionModel.MMCMotions.*;

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
public class HandGuiding extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
	}

	public void run() {
	
		//standBy position in joint space
		double a1 = Math.toRadians(0);		
		double a2 = Math.toRadians(20);
		double a3 = Math.toRadians(0);
		double a4 = Math.toRadians(-110);
		double a5 = Math.toRadians(0);
		double a6 = Math.toRadians(-40);
		double a7 = Math.toRadians(90);
			
		JointPosition J_standBy = new JointPosition(a1,a2,a3,a4,a5,a6,a7);
		
		lbr_iiwa_14_R820_1.move(ptpHome().setJointVelocityRel(0.25));
		lbr_iiwa_14_R820_1.move(ptp(J_standBy).setJointVelocityRel(0.25));
		
		lbr_iiwa_14_R820_1.move(handGuiding());
		
		lbr_iiwa_14_R820_1.move(ptpHome().setJointVelocityRel(0.25));
		
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		HandGuiding app = new HandGuiding();
		app.runApplication();
	}
}
