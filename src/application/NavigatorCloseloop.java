package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;

import matlabcontrol.MatlabConnectionException;
import matlabcontrol.MatlabInvocationException;

import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;


public class NavigatorCloseloop extends KUKADemo3 {
	private Tool tool;
	private ObjectFrame motionFrame;
	//private CartesianImpedanceControlMode impMode;
	private double distXY = 0.1;	// step size of movements
	private double distZ  = 0.1;	// step size of movements
	double k;

		 
		
	
	public NavigatorCloseloop(Tool tool, ObjectFrame frame,double state) {
		this.tool = tool;
		motionFrame = frame;
		this.k = state;
	}
	
	
	public double activate() throws MatlabConnectionException, MatlabInvocationException {
		
		matlab(4); // intialize camera and preview 
		
		
		double FM0 = matlab(0);
		double FMk,FMkminus1;
		FMkminus1 = FM0;
		tool.move(linRel(0,0,distZ,motionFrame).setJointVelocityRel(0.01));
		k=k+0.1;
        FMk = matlab(0);
        int N =13;
        
        for(int i = 0 ;i<N;i++) 
        {
        try 
        {
		if(k<3 && Navigator.matflag)
		{
		if(FMk/1E35>1)
		{
			    FMk = matlab(1);
			    tool.move(linRel(0,0,distZ,motionFrame).setJointVelocityRel(0.01));
			    double FMk1 = matlab(2);
			    if(FMk1 < FMk)
			    tool.move(linRel(0,0,-distZ,motionFrame).setJointVelocityRel(0.01));
			    
			    //tool.move(linRel(0,0,-2*distZ,motionFrame).setJointVelocityRel(0.01));
			    //FMk = matlab(3);
			    System.out.println("The system is focussed!");
			    break;
			    
		}
		if(k<0)
	    {
	    		FMkminus1 = FMk;	
	    		tool.move(linRel(0,0,4*distZ,motionFrame).setJointVelocityRel(0.01));
	    		k=k+0.4;
	    		FMk = matlab(0);	 
		}
		else if(k>2.75)
	    {
	    		FMkminus1 = FMk;	
	    		tool.move(linRel(0,0,-distZ,motionFrame).setJointVelocityRel(0.01));
	    		k=k-0.1;
	    		FMk = matlab(0);	 
		}
	    if(FMk>FMkminus1)
	    {
	    		FMkminus1 = FMk;	
	    		tool.move(linRel(0,0,distZ,motionFrame).setJointVelocityRel(0.01));
	    		k=k+0.1;
	    		FMk = matlab(0);
	    }
	    else if(FMk<FMkminus1)
	    {
	    		FMkminus1 = FMk;	
	    		tool.move(linRel(0,0,-distZ,motionFrame).setJointVelocityRel(0.01));
	    		k=k-0.1;
	    		FMk = matlab(0);
		}
		}
	
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        }
		catch (MatlabConnectionException e1) {
			// TODO Auto-generated catch block
        	e1.printStackTrace();
        	Navigator.matflag  = false;
        	return k;
			
			
		} catch (MatlabInvocationException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
			Navigator.matflag  = false;
			return k;
			
			
		}  
    	}
    	
		
        
		
             
		
		return k;
		//matlab(5); //end camera
		/*
	    isActive = true;
	    System.out.println("NavigatorCloseloop activated");
	    
	    
	    // pause robot thread
	    while(isActive) {
	    	try {
	    		Thread.sleep(500);	//
	    	} catch (InterruptedException e) {
	    		e.printStackTrace();
	    	}
	    }
	    */
	}
}

