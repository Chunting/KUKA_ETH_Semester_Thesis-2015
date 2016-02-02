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


public class Scrapedemo extends MWAtask implements Itask {
	
	// Velocities
	private static final double v_transition = 20;		// velocity during transition phase [mm/s]
	private static final double v_transition_impact = 60 ; // velocity during transition phase for impact[mm/s]
	private static final double v_contact = 200;		// velocity during contact phase [mm/s]
	private static final double v_flight = 250;			// velocity during flying phase [mm/s]
	private static final double v_taxing = 140;			// velocity for non task movements [mm/s]
	
	private static final double dz_transition = 50;		//moving in base z [mm]
	private static final double dx_contact = 220;
	private static final double dx_contact1 = 180;
	private static final double dz_refMid = 15;
	private static final double refMid = 100;
	
	private static final double x = 860;
	private static final double x1 = 390;
	private static final double y = 50;
	private static final double y1 = -60;
	
	private Frame F_contact;
	private int a;

	public Scrapedemo(LBR myLbr, Tool myTool, double force,int i) {
		// variables are inherited from class MWAtask.java
		lbr = myLbr;
		tool = myTool;
		
		force_z = force;
        a = i;
		ToolTip = tool.getFrame("/ToolTip");
		
		// get relevant robot frames
		F_flange = lbr.getFlange();		//flange frame
		F_root = lbr.getRootFrame();	//root frame
		
		if (a == 1 || a==5) //scraping short n long
		{
			F_contact 		= new Frame(F_root);
			F_contactStart 	= new Frame(F_root,x					,y, 80,Math.toRadians(180),Math.toRadians(40),Math.toRadians(180));
			F_contactEnd 	= new Frame(F_root,x-dx_contact		,y, 70,Math.toRadians(180),Math.toRadians(15),Math.toRadians(180));
			F_contactEndup  = new Frame(F_root,x-dx_contact    	,y, 75,Math.toRadians(180),Math.toRadians(15),Math.toRadians(180));
			F_flightmid     = new Frame(F_root,x-(0.5*dx_contact) ,y, 120,Math.toRadians(180),Math.toRadians(27),Math.toRadians(180));
			F_refMid        = new Frame(F_root,x-refMid			,y, 75,Math.toRadians(180),Math.toRadians(27),Math.toRadians(180));
		}
		else if (a==2) // scraping side
		{
			F_contact 		= new Frame(F_root);
			F_contactStart 	= new Frame(F_root,x					,y1, 80,Math.toRadians(180),Math.toRadians(40),Math.toRadians(180));
			F_contactEnd 	= new Frame(F_root,x-dx_contact		,y1, 70,Math.toRadians(180),Math.toRadians(15),Math.toRadians(180));
			F_contactEndup  = new Frame(F_root,x-dx_contact    	,y1, 75,Math.toRadians(180),Math.toRadians(15),Math.toRadians(180));
			F_flightmid     = new Frame(F_root,x-(0.5*dx_contact) ,y1, 120,Math.toRadians(180),Math.toRadians(27),Math.toRadians(180));
			F_refMid        = new Frame(F_root,x-refMid			,y1, 75,Math.toRadians(180),Math.toRadians(27),Math.toRadians(180));
		}
		else if (a==3)
		{
			F_contactStart 	= new Frame(F_root,x1					,y, 80,Math.toRadians(0),Math.toRadians(30),Math.toRadians(180));
			F_contactEnd 	= new Frame(F_root,x1+dx_contact1		,y, 70,Math.toRadians(0),Math.toRadians(30),Math.toRadians(180));
			F_contactEndup  = new Frame(F_root,x1+dx_contact1		,y, 80,Math.toRadians(0),Math.toRadians(30),Math.toRadians(180));
			F_flightmid     = new Frame(F_root,x1+(0.5*dx_contact1) ,y,100,Math.toRadians(0),Math.toRadians(20),Math.toRadians(180));
			
			F_refStart 	= F_contactStart.copy();
			F_refMid 	= F_contactStart.copy();
			F_refMid.transform(F_root,XyzAbcTransformation.ofTranslation(+0.9*dx_contact1,0,0));	//translation relative to lbrRoot
		}
		else if (a==4)
		{
			F_contactStart 	= new Frame(F_root,x1					,y1, 80,Math.toRadians(0),Math.toRadians(30),Math.toRadians(180));
			F_contactEnd 	= new Frame(F_root,x1+dx_contact1		,y1, 70,Math.toRadians(0),Math.toRadians(30),Math.toRadians(180));
			F_contactEndup  = new Frame(F_root,x1+dx_contact1		,y1, 80,Math.toRadians(0),Math.toRadians(30),Math.toRadians(180));
			F_flightmid     = new Frame(F_root,x1+(0.5*dx_contact1) ,y1,100,Math.toRadians(0),Math.toRadians(20),Math.toRadians(180));
			
			F_refStart 	= F_contactStart.copy();
			F_refMid 	= F_contactStart.copy();
			F_refMid.transform(F_root,XyzAbcTransformation.ofTranslation(+0.9*dx_contact1,0,0));	//translation relative to lbrRoot
		
		}

		//stop movement trigger
		condend = ForceCondition.createNormalForceCondition(ToolTip,F_root, CoordinateAxis.Z, force_z); 
	}
	
	
	
	public void goCycle(int imp) {
		
		if (firstCycle) {
			tool.move(lin(F_contactStart).setCartVelocity(v_taxing).setCartAcceleration(200));
			
			// START THE CYCLE
			// PHASE 1: transition, landing
			System.out.println("start landing, impedance mode ");
			System.out.println(String.format("before landing: Z = %f", lbr.getCurrentCartesianPosition(ToolTip, F_root).getZ()));
	
			if(imp == 0)
			tool.move(linRel(0,0,-dz_transition,F_root).breakWhen(condend).setCartVelocity(v_transition).setMode(cartImpMode));
			else
			tool.move(linRel(0,0,-dz_transition,F_root).breakWhen(condend).setCartVelocity(v_transition_impact).setMode(cartImpMode));
		}
		
		F_contact = lbr.getCurrentCartesianPosition(ToolTip,F_root);

		F_contact.setZ( lbr.getCurrentCartesianPosition(ToolTip,F_root).getZ()-2);
		
		// build trajectory of contact phase
		data = lbr.getExternalForceTorque(ToolTip,F_root);
		System.out.println(String.format("after landing: Fz = %1$f ",data.getForce().getZ()));
		
		F_refMid.setZ(lbr.getCurrentCartesianPosition(ToolTip,F_root).getZ()-dz_refMid);
		System.out.println(String.format("F_refMid x: %1$f y: %2$f z: %3$f", F_refMid.getX(), F_refMid.getY(), F_refMid.getZ()));
		
		Spline cycle;
		
		if(lastCycle)
		{
			if(a==1 || a==2)
			{
			       cycle = new Spline (
						//lin(F_contact).breakWhen(condend).setCartVelocity(v_transition),
						lin(F_refMid).setCartVelocity(v_contact),		// must be kept lin, otherwise landing is splined too !!!
						spl(F_contactEnd).setCartVelocity(v_contact),//.setCartAcceleration(100),
						lin(F_contactEndup).setCartVelocity(v_flight),//.setCartAcceleration(100),
						spl(F_flightmid).setCartVelocity(v_flight),//setCartAcceleration(100),
						spl(F_contactStart).setCartVelocity(v_flight)//.setCartAcceleration(100),
						
				).setOrientationType(SplineOrientationType.OriJoint);
			}
			else if (a==5)
			{
				 cycle = new Spline (
							//lin(F_contact).breakWhen(condend).setCartVelocity(v_transition),
							lin(F_refMid).setCartVelocity(v_contact),		// must be kept lin, otherwise landing is splined too !!!
							spl(F_contactEnd).setCartVelocity(v_contact),//.setCartAcceleration(100),
							lin(F_contactEndup).setCartVelocity(v_flight),//.setCartAcceleration(100),
							spl(F_flightmid).setCartVelocity(v_flight),//setCartAcceleration(100),
							spl(F_contactStart).setCartVelocity(v_flight)//.setCartAcceleration(100),
							
					).setOrientationType(SplineOrientationType.OriJoint);
			}
			else
			{
			    cycle = new Spline (
						lin(F_refMid).setCartVelocity(v_contact),		// must be kept lin, otherwise landing is splined too !!!
						spl(F_contactEnd).setCartVelocity(v_contact),//.setCartAcceleration(100),
						spl(F_contactEndup).setCartVelocity(v_flight),//.setCartAcceleration(100),
						spl(F_flightmid).setCartVelocity(v_flight),
						spl(F_contactStart).setCartVelocity(v_flight)//.setCartAcceleration(100)
						
				).setOrientationType(SplineOrientationType.Variable);
			}	
		}
		else
		{
		if(a==1 || a==2)
		{
		       cycle = new Spline (
					//lin(F_contact).breakWhen(condend).setCartVelocity(v_transition),
					lin(F_refMid).setCartVelocity(v_contact),		// must be kept lin, otherwise landing is splined too !!!
					spl(F_contactEnd).setCartVelocity(v_contact),//.setCartAcceleration(100),
					lin(F_contactEndup).setCartVelocity(v_flight),//.setCartAcceleration(100),
					spl(F_flightmid).setCartVelocity(v_flight),//setCartAcceleration(100),
					spl(F_contactStart).setCartVelocity(v_flight),//.setCartAcceleration(100),
					spl(F_contact).setCartVelocity(v_transition_impact)	
			).setOrientationType(SplineOrientationType.OriJoint);
		}
		else if (a==5)
		{
			 cycle = new Spline (
						//lin(F_contact).breakWhen(condend).setCartVelocity(v_transition),
						lin(F_refMid).setCartVelocity(v_contact),		// must be kept lin, otherwise landing is splined too !!!
						spl(F_contactEnd).setCartVelocity(v_contact),//.setCartAcceleration(100),
						lin(F_contactEndup).setCartVelocity(v_flight),//.setCartAcceleration(100),
						spl(F_flightmid).setCartVelocity(v_flight),//setCartAcceleration(100),
						spl(F_contactStart).setCartVelocity(v_flight),//.setCartAcceleration(100),
						spl(F_contact).setCartVelocity(v_transition_impact)	
				).setOrientationType(SplineOrientationType.OriJoint);
		}
		else
		{
		    cycle = new Spline (
					lin(F_refMid).setCartVelocity(v_contact),		// must be kept lin, otherwise landing is splined too !!!
					spl(F_contactEnd).setCartVelocity(v_contact),//.setCartAcceleration(100),
					spl(F_contactEndup).setCartVelocity(v_flight),//.setCartAcceleration(100),
					spl(F_flightmid).setCartVelocity(v_flight),
					spl(F_contactStart).setCartVelocity(v_flight),//.setCartAcceleration(100)
					spl(F_contact).setCartVelocity(v_transition_impact)	
			).setOrientationType(SplineOrientationType.Variable);
		}	
		}
			
		
		// PHASE 2 and 3: contact and flight
		System.out.println("start contact phase, impedance mode");
		tool.move(cycle.setMode(cartImpMode));
		
		firstCycle = false; 	// subsequent tasks are executed in different fashion; 
		
	}
}

