package application;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

public class PassiveMotion {

	private LBR lbr;
	
	private static final int stiffnessZ = 100;
	private static final int stiffnessY = 4000;
	private static final int stiffnessX = 4000;
	private static final int stiffnessA = 200;
	private static final int stiffnessB = 200;
	private static final int stiffnessC = 200;
	
	private CartesianImpedanceControlMode passive =	new CartesianImpedanceControlMode();
	private IMotionContainer positionHoldContainer;

	
	public PassiveMotion(LBR myLbr){
		
		lbr = myLbr;		
		
		passive.parametrize(CartDOF.X).setStiffness(stiffnessX);
		passive.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		passive.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
		passive.parametrize(CartDOF.A).setStiffness(stiffnessA);
		passive.parametrize(CartDOF.B).setStiffness(stiffnessB);
		passive.parametrize(CartDOF.C).setStiffness(stiffnessC);
	}

	public void activate() {
		if (lbr.isReadyToMove()) {
			positionHoldContainer = lbr.moveAsync(new PositionHold(passive, -1, null));
		}
		else {
			System.out.println("Robot not ready for passive motion");
		}
	}
	
	public void terminate() {
		positionHoldContainer.cancel();
	}
		
}
