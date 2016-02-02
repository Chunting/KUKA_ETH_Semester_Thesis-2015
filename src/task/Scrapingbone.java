package task;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.spl;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;


public class Scrapingbone extends MWAtask implements Itask {
	
	// Velocities
	private static final double v_transition = 20;		// velocity during transition phase [mm/s]
	private static final double v_transition_impact = 40 ; // velocity during transition phase for impact[mm/s]
	private static final double v_contact = 100;		// velocity during contact phase [mm/s]
	private static final double v_flight = 200;			// velocity during flying phase [mm/s]
	private static final double v_taxing = 140;			// velocity for non task movements [mm/s]
	
	private static final double dz_transition = 50;		//moving in base z [mm]
	private static final double dx_contact = 120;
	private static final double dz_refMid = 15;
	private static final double refMid = 48;
	

	public Scrapingbone(LBR myLbr, Tool myTool, double force) {
		// variables are inherited from class MWAtask.java
		lbr = myLbr;
		tool = myTool;
		
		force_z = force;
		
		ToolTip = tool.getFrame("/ToolTip");
		
		// get relevant robot frames
		F_flange = lbr.getFlange();		//flange frame
		F_root = lbr.getRootFrame();	//root frame
		
		F_contactStart 	= new Frame(F_root,823					,-65, 130,Math.toRadians(0),Math.toRadians(-30),Math.toRadians(180));
		F_contactEnd 	= new Frame(F_root,823-dx_contact		,-65, 90,Math.toRadians(0),Math.toRadians(-20),Math.toRadians(180));
		F_contactEndup  = new Frame(F_root,823-dx_contact		,-65, 120,Math.toRadians(0),Math.toRadians(-20),Math.toRadians(180));
		F_flightmid     = new Frame(F_root,823-(0.5*dx_contact) ,-65, 140,Math.toRadians(0),Math.toRadians(-25),Math.toRadians(180));
		F_refMid        = new Frame(F_root,823-refMid			,-65, 100,Math.toRadians(0),Math.toRadians(-25),Math.toRadians(180));
		F_refStart 	= F_contactStart.copy();
		//F_refMid 	= F_contactStart.copy();
		//F_refMid.transform(F_root,XyzAbcTransformation.ofTranslation(+0.3*dx_contact,0,0));	//translation relative to lbrRoot
		
		//stop movement trigger
		condend = ForceCondition.createNormalForceCondition(ToolTip,F_root, CoordinateAxis.Z, force_z); 
	}
	
	
	
	public void goCycle(int imp) {
		
		tool.move(lin(F_contactStart).setCartVelocity(v_taxing).setCartAcceleration(200));
		
		// START THE CYCLE
		// PHASE 1: transition, landing
		System.out.println("start landing, impedance mode ");
		System.out.println(String.format("before landing: Z = %f", lbr.getCurrentCartesianPosition(ToolTip, F_root).getZ()));

		if(imp == 0)
		tool.move(linRel(0,0,-dz_transition,F_root).breakWhen(condend).setCartVelocity(v_transition).setMode(cartImpMode));
		else
		tool.move(linRel(0,0,-dz_transition,F_root).breakWhen(condend).setCartVelocity(v_transition_impact).setMode(cartImpMode));
		
		// build trajectory of contact phase
		data = lbr.getExternalForceTorque(ToolTip,F_root);
		System.out.println(String.format("after landing: Fz = %1$f ",data.getForce().getZ()));
		
		//F_refMid.setZ(lbr.getCurrentCartesianPosition(ToolTip,F_root).getZ()-dz_refMid);
		System.out.println(String.format("F_refMid x: %1$f y: %2$f z: %3$f", F_refMid.getX(), F_refMid.getY(), F_refMid.getZ()));
		
		Spline cycle = new Spline (
				lin(F_refMid).setCartVelocity(v_contact),		// must be kept lin, otherwise landing is splined too !!!
				spl(F_contactEnd).setCartVelocity(v_contact).setCartAcceleration(100),
				spl(F_contactEndup).setCartVelocity(v_flight).setCartAcceleration(100),
				spl(F_flightmid).setCartVelocity(v_flight),
				spl(F_contactStart).setCartVelocity(v_flight).setCartAcceleration(100)
		).setOrientationType(SplineOrientationType.OriJoint);
		
		// PHASE 2 and 3: contact and flight
		System.out.println("start contact phase, impedance mode");
		tool.move(cycle.setMode(cartImpMode));
		
	}
}
