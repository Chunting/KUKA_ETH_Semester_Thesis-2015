package task;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

import task.MountingPosition;

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
public class Microscopy extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	
	double a1 = Math.toRadians(40);		// joint angles
	double a2 = Math.toRadians(68);
	double a3 = Math.toRadians(0);
	double a4 = Math.toRadians(-94);
	double a5 = Math.toRadians(42);
	double a6 = Math.toRadians(102);
	double a7 = Math.toRadians(-35);
	
	int UI_focus;
	JointPosition J_micro =  new JointPosition(a1,a2,a3,a4,a5,a6,a7);

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");	
		
	}
 
	
	public void run() {
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
			 		case 0: lbr_iiwa_14_R820_1.move(linRel(0.5,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
			 				break;
			 		case 1: lbr_iiwa_14_R820_1.move(linRel(0,0.5,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
			 				break;
			 		case 2: lbr_iiwa_14_R820_1.move(linRel(0,0,0.1,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
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
			 		case 0: lbr_iiwa_14_R820_1.move(linRel(-0.5,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
			 				break;
			 		case 1: lbr_iiwa_14_R820_1.move(linRel(0,-0.5,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
			 				break;
			 		case 2: lbr_iiwa_14_R820_1.move(linRel(0,0,-0.1,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
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
		    
		    
		
		lbr_iiwa_14_R820_1.move(lin(F_micro1).setJointVelocityRel(0.25));	
	//	lbr_iiwa_14_R820_1.move(ptp(J_micro).setJointVelocityRel(0.25));
		lbr_iiwa_14_R820_1.move(linRel(-150,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.1));
		lbr_iiwa_14_R820_1.move(linRel(-50,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
	   
		while(true)
		{
			UI_focus  = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Do you want to focus", "X","Y","Z","No");
			if (UI_focus == 3) 
		    break;
		}
		
		
		getApplicationControl().pause();
		
		lbr_iiwa_14_R820_1.move(linRel(50,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.01));
		lbr_iiwa_14_R820_1.move(linRel(100,0,0,lbr_iiwa_14_R820_1.getRootFrame()).setJointVelocityRel(0.1));
		
		
		MountingPosition app = new MountingPosition();
		app.runApplication();
		
		
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		Microscopy app = new Microscopy();
		app.runApplication();
	}
}
