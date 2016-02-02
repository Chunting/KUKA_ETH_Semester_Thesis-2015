package josMovements;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

//import com.kuka.roboticsAPI.conditionModel.MotionPathCondition;
//import com.kuka.roboticsAPI.conditionModel.ReferenceType;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
//import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
//import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
//import com.kuka.roboticsAPI.sensorModel.DataRecorder.AngleUnit;
//import com.kuka.roboticsAPI.sensorModel.StartRecordingAction;

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

//File stored in Robotersteuerung in C:\KRC\Roboter\Log\DataRecorder
public class DataRecorder_test1 extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	private ObjectFrame F_flange;		//flange frame
	private ObjectFrame F_root;			//root frame
	
	final static double radiusOfCircMove=120;

	final static double offsetAxis2And4=Math.toRadians(20);
	final static double offsetAxis4And6=Math.toRadians(-40);
	double[] loopCenterPosition= new double[]{
			0, offsetAxis2And4, 0, offsetAxis2And4 +offsetAxis4And6 -Math.toRadians(90), 0, offsetAxis4And6,Math.toRadians(90)};
	
	public DataRecorder rec;

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
		
		F_flange = lbr_iiwa_14_R820_1.getFlange();		//flange frame
		F_root = lbr_iiwa_14_R820_1.getRootFrame();		//root frame
	}

	public void run() {
		lbr_iiwa_14_R820_1.move(ptpHome());
		
		//move to center of circular position
		PTP ptpToLoopCenter = ptp(loopCenterPosition);
		ptpToLoopCenter.setJointVelocityRel(0.25);
		lbr_iiwa_14_R820_1.move(ptpToLoopCenter);
		
		Frame startFrame = lbr_iiwa_14_R820_1.getCurrentCartesianPosition(lbr_iiwa_14_R820_1.getFlange());
		//getApplicationData().getProcessData("startFrame").setValue(startFrame));
		
		System.out.println("bla bla");
		
		DataRecorder rec = new DataRecorder(); //data recording 
		rec.setFileName("Recording.log");
		rec.setSampleRate(100);
		rec.addCartesianForce(F_flange,F_root);
		rec.addCurrentCartesianPositionXYZ(F_flange, F_root);
		
		rec.enable();
		rec.startRecording();
		System.out.println("recording activated");
		
	/*	rec = new DataRecorder(); //data recording 
		DataRecorder rec = new DataRecorder(); //data recording 
		rec.setFileName("Recording.log");
		rec.setSampleRate(100);
		rec.addCartesianForce(F_flange,F_root);
		rec.addCurrentCartesianPositionXYZ(F_flange, F_root);
		
		rec.enable();
		boolean enabling = rec.isEnabled();
		System.out.println("recording enabled" + enabling);
		rec.startRecording();
		boolean recording = rec.isRecording();
		System.out.println("recording activated" + recording);
		if(rec.isFileAvailable())
		System.out.println("data file available");
		System.out.println(rec.getURL().toString());
		
		// experiment 
		for (int i_cycle = 1; i_cycle <= 3; i_cycle++) {
			System.out.println(String.format("cycle # %1$d", i_cycle)); */
			
			//Trajektorie fahren
			//lbr_iiwa_14_R820_2.move(lin(startFrame));
			Spline lemniscateSpline = createLemniscateSpline(startFrame).setJointJerkRel(0.5).setCartVelocity(250);
			lemniscateSpline.setJointVelocityRel(0.25);
			lbr_iiwa_14_R820_1.move(lemniscateSpline);
		
		
	// stop recording
	rec.stopRecording();
	System.out.println("data recording stopped");	
	}
	
	private Spline createLemniscateSpline(Frame centerFrame) {
		// Create a new frame with the center frame as parent. Set an offset for the x axis to this parent.
		Frame rightFrame=(new Frame(centerFrame)).setX(2*radiusOfCircMove);
		// Create a new frame with the center frame as parent. Set an offset for the x axis to this parent.
		Frame leftFrame= (new Frame(centerFrame)).setX(-2*radiusOfCircMove);	
		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame topLeftFrame= (new Frame(centerFrame)).setX(-radiusOfCircMove).setY(radiusOfCircMove);		
		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame topRightFrame= (new Frame(centerFrame)).setX(+radiusOfCircMove).setY(radiusOfCircMove);		
		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame bottomRightFrame= (new Frame(centerFrame)).setX(+radiusOfCircMove).setY(-radiusOfCircMove);
		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame bottomLeftFrame= (new Frame(centerFrame)).setX(-radiusOfCircMove).setY(-radiusOfCircMove);
		// Create a spline that describes a lemniscate
		Spline spline = new Spline(
			spl(bottomLeftFrame),
			spl(leftFrame),
			spl(topLeftFrame),
			spl(centerFrame),
			spl(bottomRightFrame),
			spl(rightFrame),
			spl(topRightFrame),
			spl(centerFrame));
		return spline;
}

	
	

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		DataRecorder_test1 app = new DataRecorder_test1();
		app.runApplication();
	}
}
