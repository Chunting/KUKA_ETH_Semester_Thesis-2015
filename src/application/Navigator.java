package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.util.concurrent.TimeUnit;

import javax.swing.*;

import matlabcontrol.MatlabConnectionException;
import matlabcontrol.MatlabInvocationException;

import Arduino.Stepper;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.KROSMotion;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;


public class Navigator extends KUKADemo3 {
	private Tool tool;
	private LBR lbr;
	private ObjectFrame motionFrame;
	//private CartesianImpedanceControlMode impMode;
	private double distXY = 0.3;	// step size of movements
	private double distZ  = 0.1;	// step size of movements
	private double rad  = Math.toRadians(2);	// step size of movement
	private boolean isActive = false;	// pause flag
	
	Frame F_Focus;
	ObjectFrame ToolTip;
	Stepper step = new Stepper(); //Stepper motor control class
	
	// Erzeugung eines neuen Dialoges
    private JDialog meinJDialog = new JDialog();
	private JPanel panel = new JPanel();
   
    private InputMap im = panel.getInputMap(JPanel.WHEN_IN_FOCUSED_WINDOW);
    private ActionMap am = panel.getActionMap();
    
    CartesianImpedanceControlMode impedanceControlMode = 	new CartesianImpedanceControlMode();
    PositionHold positionHold = new PositionHold(impedanceControlMode, 10, TimeUnit.SECONDS);
    IMotionContainer positionHoldContainer;
  //  ICondition condend = ForceCondition.createNormalForceCondition(tool.getRootFrame(),motionFrame, CoordinateAxis.Z, 5); 
    
    double state = 0;
    NavigatorCloseloop navigator;
    static boolean matflag = true ;
	int i = 0;	
	private class KeyAction extends AbstractAction {
		private static final long serialVersionUID = 1L;
		private String cmd;

	    public KeyAction(String cmd) {
	        this.cmd = cmd;
	    }

	    @Override
	    public void actionPerformed(ActionEvent e) {
	        if (cmd.equalsIgnoreCase("LeftArrow")) {
	            System.out.println("The left arrow was pressed!");
	            if(i==1)
	            positionHoldContainer.cancel();	
	            i=0;
	            tool.move(linRel(0,distXY,0,motionFrame).setJointVelocityRel(0.01));
	            
	        } else if (cmd.equalsIgnoreCase("RightArrow")) {
	            System.out.println("The right arrow was pressed!");
	            if(i==1)
		            positionHoldContainer.cancel();
	            i=0;
	            tool.move(linRel(0,-distXY,0,motionFrame).setJointVelocityRel(0.01));
	            
	        } else if (cmd.equalsIgnoreCase("UpArrow")) {
	            System.out.println("The up arrow was pressed!");
	            if(i==1)
		            positionHoldContainer.cancel();
	            i=0;
	            tool.move(linRel(distXY,0,0,motionFrame).setJointVelocityRel(0.01));
	            
	        } else if (cmd.equalsIgnoreCase("DownArrow")) {
	            System.out.println("The down arrow was pressed!");
	            if(i==1)
		            positionHoldContainer.cancel();
	            i=0;
	            tool.move(linRel(-distXY,0,0,motionFrame).setJointVelocityRel(0.01));
	            
	        } else if (cmd.equalsIgnoreCase("PageUp")) {
	            System.out.println("The page up was pressed!");
	            if(i==1)
		            positionHoldContainer.cancel();
	            i=0;
	            tool.move(linRel(0,0,distZ,motionFrame).setJointVelocityRel(0.01));
	            state = state + 0.1;
	            
	        } else if (cmd.equalsIgnoreCase("PageDown")) {
	            System.out.println("The page down was pressed!");
	            if(i==1)
		            positionHoldContainer.cancel();
	            i=0;
	            tool.move(linRel(0,0,-distZ,motionFrame).setJointVelocityRel(0.01));  
	            state = state-0.1;
	            
	        }  else if (cmd.equalsIgnoreCase("A")) {
	            System.out.println("A was pressed!");
	            if(i==1)
		            positionHoldContainer.cancel();
	            i=0;
	            tool.move(linRel(0,0,0,rad,0,0,motionFrame).setJointVelocityRel(0.01));  
	            
	        } else if (cmd.equalsIgnoreCase("S")) {
	            System.out.println("S was pressed!");
	            if(i==1)
		            positionHoldContainer.cancel();
	            i=0;
	            tool.move(linRel(0,0,0,0,rad,0,motionFrame).setJointVelocityRel(0.01));  
	            
	        } else if (cmd.equalsIgnoreCase("D")) {
	            System.out.println("D was pressed!");
	            if(i==1)
		            positionHoldContainer.cancel();
	            i=0;
	            tool.move(linRel(0,0,0,0,0,rad,motionFrame).setJointVelocityRel(0.01));  
	            
	        } else if (cmd.equalsIgnoreCase("Y")) {
	            System.out.println("Y down was pressed!");
	            if(i==1)
		            positionHoldContainer.cancel();
	            i=0;
	            tool.move(linRel(0,0,0,-rad,0,0,motionFrame).setJointVelocityRel(0.01));  
	            
	        } else if (cmd.equalsIgnoreCase("X")) {
	            System.out.println("X was pressed!");
	            if(i==1)
		            positionHoldContainer.cancel();
	            i=0;
	            tool.move(linRel(0,0,0,0,-rad,0,motionFrame).setJointVelocityRel(0.01));  
	            
	        } else if (cmd.equalsIgnoreCase("C")) {
	            System.out.println("C was pressed!");
	            if(i==1)
		            positionHoldContainer.cancel();
	            i=0;
	            tool.move(linRel(0,0,0,0,0,-rad,motionFrame).setJointVelocityRel(0.01));  
	            
	        }  else if (cmd.equalsIgnoreCase("G")) {
	            System.out.println("G was pressed. Goin gto autofocus position!");
	            if(i==1)
		            positionHoldContainer.cancel();
	            i=0;
	            tool.move(lin(F_Focus).setJointVelocityRel(0.01));  
	            
	        } 
	    	   else if (cmd.equalsIgnoreCase("Q")) {
	    		System.out.println("Q was pressed!");
	    		// The robot is set to position hold and impedance control mode gets activated without a timeout.
	    		if(i==0)
	    		positionHoldContainer = tool.moveAsync(positionHold);//.breakWhen(condend));
	    		i=1;
	    		step.sendData("y"); // go forward
            
        	} else if (cmd.equalsIgnoreCase("W")) {
        		System.out.println("W was pressed!");
        		if(i==0)
        		positionHoldContainer = tool.moveAsync(positionHold);//.breakWhen(condend));
        		i=1;
        		step.sendData("n"); // go backward
         
     		} else if (cmd.equalsIgnoreCase("E")) {
	    		System.out.println("E was pressed!");
	    		
	    		if(i==0)
		            positionHoldContainer.cancel();
	    		i=1;
	    		step.sendData("y"); // go forward
            
        	} else if (cmd.equalsIgnoreCase("R")) {
        		System.out.println("R was pressed!");
        		if(i==0)
    	            positionHoldContainer.cancel();
        		i=1;
        		step.sendData("n"); // go backward
         
     		}else if (cmd.equalsIgnoreCase("F")) {
	            System.out.println("Focus measure!");
	            try {
					matlab(1);
				} catch (MatlabConnectionException e1) {
					// TODO Auto-generated catch block
					matflag= false;
					e1.printStackTrace();
				} catch (MatlabInvocationException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
					matflag = false;
				}       
	            F_Focus = lbr.getCurrentCartesianPosition(ToolTip,motionFrame);  //storing the focussed frame. Robot goes to this position when G is pressed
	        }
     		
     		else if (cmd.equalsIgnoreCase("N")) {
        		System.out.println("N was pressed!");
        		if(i==1)
    	            positionHoldContainer.cancel();
        		i=0;
        		
        			try {
        				state = navigator.activate();
        			} catch (MatlabConnectionException e1) {
        				// TODO Auto-generated catch block
        				matflag = false;
        				e1.printStackTrace();
        			} catch (MatlabInvocationException e1) {
        				// TODO Auto-generated catch block
        				matflag = false;
        				e1.printStackTrace();
        			}	
        		    
     		} 
     		
     		
     		else if (cmd.equalsIgnoreCase("Enter")) {
	        	System.out.println("Navigater terminated");
	        	meinJDialog.dispose();
	        	// end stepper control
	        	step.close();
	        	if(matflag)
	        	{
	        	try {
					matlab(5); //end camera
				} catch (MatlabConnectionException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				} catch (MatlabInvocationException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
	        	}
	        	if(i==1)
		            positionHoldContainer.cancel();
	            i=0;
	        	
	        	
	        	isActive = false;	// continue robot thread
	        }
     		
	    }
	}
	
	// constructor takes tool that is to be navigated
	public Navigator(Tool tool, ObjectFrame frame, LBR mylbr) {
		this.tool = tool;
		motionFrame = frame;
		lbr = mylbr;
		
	}


	public void activate() {
		
		System.out.println("Activating Navigator");
		meinJDialog.setTitle("Navigator");
		meinJDialog.setBounds(500,300,200,200);
		meinJDialog.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		meinJDialog.add(panel);
		meinJDialog.setVisible(true);
		    
		panel.setBackground(Color.red);
		
		im.put(KeyStroke.getKeyStroke(KeyEvent.VK_RIGHT, 0), "RightArrow");
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_LEFT, 0), "LeftArrow");
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_UP, 0), "UpArrow");
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_DOWN, 0), "DownArrow");
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "Enter");
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_PAGE_UP, 0), "PageUp");
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_PAGE_DOWN, 0), "PageDown");
	    
	    
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_A, 0), "A");//A
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_S, 0), "S");//B
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_D, 0), "D");//C
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_Y, 0), "Y");//-A
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_X, 0), "X");//-B
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_C, 0), "C");//-C
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_F, 0), "F"); // storing the focus position
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_G, 0), "G");//Autofocus position
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_N, 0), "N");//Navigator closeloop
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_Q, 0), "Q"); // stepper up
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_W, 0), "W"); // stepper down
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_E, 0), "E"); // stepper without impedance
	    im.put(KeyStroke.getKeyStroke(KeyEvent.VK_R, 0), "R"); //stepper wihtout impedance
	   
	    
	    am.put("RightArrow", new KeyAction("RightArrow"));
	    am.put("LeftArrow", new KeyAction("LeftArrow"));
	    am.put("UpArrow", new KeyAction("UpArrow"));
	    am.put("DownArrow", new KeyAction("DownArrow"));
	    am.put("Enter", new KeyAction("Enter"));    
	    am.put("PageUp", new KeyAction("PageUp"));
	    am.put("PageDown", new KeyAction("PageDown"));
	    
	    
	    am.put("A", new KeyAction("A"));
	    am.put("S", new KeyAction("S"));
	    am.put("D", new KeyAction("D"));
	    am.put("Y", new KeyAction("Y"));
	    am.put("X", new KeyAction("X"));
	    am.put("C", new KeyAction("C"));
	    am.put("F", new KeyAction("F"));
	    am.put("G", new KeyAction("G"));
	    am.put("N", new KeyAction("N"));
	    am.put("Q", new KeyAction("Q"));
	    am.put("W", new KeyAction("W"));
	    am.put("E", new KeyAction("E"));
	    am.put("R", new KeyAction("R"));
	   
	    
		System.out.println("Initializing stepper");
	    step.initialize(); //initializing stepper motor control via java
	/*    
	    try {
			matlab(4);
		} catch (MatlabConnectionException e1) {
			// TODO Auto-generated catch block
			matflag = false ;
			e1.printStackTrace();
		} catch (MatlabInvocationException e1) {
			// TODO Auto-generated catch block
			matflag = false ;
			e1.printStackTrace();
		} // intialize camera and preview */
	    
		System.out.println("Setting impedance mode");
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(1000);
		impedanceControlMode.parametrize(CartDOF.ALL).setDamping(0.9);
		
		navigator = new NavigatorCloseloop(tool, motionFrame,state);
		
	    isActive = true;
	    System.out.println("Navigator activated");
	   ToolTip = tool.getFrame("/ToolTip");
	   F_Focus = lbr.getCurrentCartesianPosition(ToolTip,motionFrame);
	    
	    // pause robot thread
	   
	    while(isActive) {
	    	try {
	    		Thread.sleep(20000);	//
	    	} catch (InterruptedException e) {
	    		e.printStackTrace();
	    	}
	    	
	    	System.out.println("Navigater terminated");
        	meinJDialog.dispose();
        	// end stepper control
        	step.close();
        	if(matflag)
        	{
        	try {
				matlab(5); //end camera
			} catch (MatlabConnectionException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			} catch (MatlabInvocationException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
        	}
        	if(i==1)
	            positionHoldContainer.cancel();
            i=0;
        	
        	
        	isActive = false;	// continue robot thread*/
	    }
	    
	}
}

