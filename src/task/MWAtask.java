package task;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;

abstract public class MWAtask {

	protected LBR lbr;
	protected Tool tool;
	
	protected CartesianImpedanceControlMode cartImpMode;
	
	protected ForceCondition condend;
	
	protected ForceSensorData data;
	
	protected ObjectFrame F_root;
	protected ObjectFrame F_flange;
	protected ObjectFrame ToolTip;
	
	// Trajectory Frames
	protected Frame F_contactEnd;
	protected Frame F_contactStart;
	protected Frame F_refStart;
	protected Frame F_refMid;
	protected Frame F_flightmid;
	protected Frame F_contactEndup;
		
	protected double force_z;
	
	public boolean firstCycle = true;	// flag needed if first cycles differs from fellows
	public boolean lastCycle = false;	// flag needed if first cycles differs from fellows

	
	protected MWAtask() {
		// set-up the impedance mode
		cartImpMode = new CartesianImpedanceControlMode();
			
		cartImpMode.parametrize(CartDOF.X,CartDOF.Y).setStiffness(4000);  // max 5000
		cartImpMode.parametrize(CartDOF.Z).setStiffness(2500);
		cartImpMode.parametrize(CartDOF.TRANSL).setDamping(0.8);
	
		cartImpMode.parametrize(CartDOF.ROT).setStiffness(280);		// max 300
		cartImpMode.parametrize(CartDOF.ROT).setDamping(0.9);
	 
		cartImpMode.setNullSpaceStiffness(1000);
	
		cartImpMode.setMaxControlForce(70, 70, 70, 10, 10, 10, true);
		
	}
	
	abstract public void goCycle(int imp);	
		
}
