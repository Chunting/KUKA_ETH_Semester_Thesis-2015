package task;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;

public class Sawing1 extends MWAtask implements Itask {
	
	private static final double v_taxingRel = 0.15;		// relative velocity for non task movements [1]
	private static final double v_transition1 = 100;		// velocity during transition phase [mm/s]
	private static final double v_transition2 = 10 ; // velocity during transition phase for impact[mm/s]
	private static final double v_forward = 200;		// velocity during contact phase [mm/s]
	private static final double v_backward = 200;			// velocity during flying phase [mm/s]		
	int numCycles;
	
	
	double a1 = Math.toRadians(-4.57);		
	double a2 = Math.toRadians(26.21);
	double a3 = Math.toRadians(0.85);
	double a4 = Math.toRadians(-114.23);
	double a5 = Math.toRadians(8.19);
	double a6 = Math.toRadians(-54.96);
	double a7 = Math.toRadians(-6.67);
	
    JointPosition J_saw1 = new JointPosition(a1,a2,a3,a4,a5,a6,a7);
    
    private static final double dz_transition1 = 250;		//moving in base z [mm
    private static final double dz_transition2 = 50;		//moving in base z [mm
    private static final double dx_stroke = 25; //stroke length in mm
   
    
	
	public Sawing1(LBR myLbr, Tool myTool, double force, int strokes)
	{
		lbr = myLbr;
		tool = myTool;
		
		force_z = force;
		
	    numCycles =  strokes ;
		
		ToolTip = tool.getFrame("/ToolTip");
		
		// get relevant robot frames
		F_flange = lbr.getFlange();		//flange frame
		F_root = lbr.getRootFrame();	//root frame
		
		condend = ForceCondition.createNormalForceCondition(ToolTip,F_root, CoordinateAxis.Z, force_z); 

	
	}
	
	public void goCycle(int imp) {
		
		System.out.println("going to sawing position 1");
		tool.move(ptp(J_saw1).setJointVelocityRel(v_taxingRel));
		
		tool.move(linRel(0,0,-dz_transition1,F_root).setCartVelocity(v_transition1).setMode(cartImpMode));
		
		tool.move(linRel(0,0,-dz_transition2,F_root).breakWhen(condend).setCartVelocity(v_transition2).setMode(cartImpMode));
		tool.move(linRel(15,0,0,F_root).breakWhen(condend).setCartVelocity(v_transition2).setMode(cartImpMode));
		
		Frame F_start = lbr.getCurrentCartesianPosition(ToolTip,F_root);
		Frame F_end   = lbr.getCurrentCartesianPosition(ToolTip,F_root);
		double X = F_start.getX();
		F_end.setX(lbr.getCurrentCartesianPosition(ToolTip,F_root).getX()-25);
		
		
		for (int i_cycle = 1; i_cycle <= numCycles; i_cycle++) {
			System.out.println(String.format("cycle # %1$d", i_cycle));
			
			
			Frame F_mid = lbr.getCurrentCartesianPosition(ToolTip,F_root);
			double X_mid = F_mid.getX();
			
			if (X_mid<X)
			tool.move(linRel(X-X_mid,0,0).setCartVelocity(v_forward).setMode(cartImpMode).setOrientationType(SplineOrientationType.Constant));
				
				
			//F_start.setX(lbr.getCurrentCartesianPosition(ToolTip,F_root).getX()+5);
			
			//F_start.setY(lbr.getCurrentCartesianPosition(ToolTip,F_root).getY()-2);
			//tool.moveAsync(linRel(0,0,-3,F_root).breakWhen(condend).setCartVelocity(v_transition2).setMode(cartImpMode));
			//tool.move(linRel(-dx_stroke,0,-5,F_root).setCartVelocity(v_backward).setMode(cartImpMode).setOrientationType(SplineOrientationType.Constant));
			//tool.move(linRel(dx_stroke,0,1,F_root).setCartVelocity(v_forward).setMode(cartImpMode).setOrientationType(SplineOrientationType.Constant));
			tool.move(lin(F_end).setCartVelocity(v_forward).setMode(cartImpMode).setOrientationType(SplineOrientationType.Constant));
			tool.move(lin(F_start).setCartVelocity(v_forward).setMode(cartImpMode).setOrientationType(SplineOrientationType.Constant));
			
			if(lbr.getExternalForceTorque(ToolTip,F_root).getForce().getZ()<30)
			{
			F_end.setZ(lbr.getCurrentCartesianPosition(ToolTip,F_root).getZ()-2);
			F_start.setZ(lbr.getCurrentCartesianPosition(ToolTip,F_root).getZ()-2);
			}

		}
		tool.move(linRel(0,0,dz_transition1,F_root).setCartVelocity(v_transition1).setMode(cartImpMode));
		

	}

}
