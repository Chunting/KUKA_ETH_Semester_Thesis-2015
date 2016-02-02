package josMovements;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;

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
public class showTools extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	
	private Frame F_air,myFrame;
	private Tool ScraperMedium;

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
	}

	public void run() {
		F_air = getApplicationData().getFrame("/F_air").copy();
		ScraperMedium = getApplicationData().createFromTemplate("ScraperMedium");
		
		ObjectFrame lbrRoot = lbr_iiwa_14_R820_1.getRootFrame();	//root frame
		myFrame = new Frame(lbrRoot,480,-300,0,Math.toRadians(-180),Math.toRadians(0),Math.toRadians(180));
		
		ScraperMedium.attachTo(lbr_iiwa_14_R820_1.getFlange());
		
		System.out.println(String.format("F_air x %1$f, y %2$f z %3$f", F_air.getX(), F_air.getY(), F_air.getZ()));
		System.out.println(String.format("ScraperMedium load: %1$f", ScraperMedium.getLoadData().getMass()));
		
		lbr_iiwa_14_R820_1.move(ptp(F_air).setJointVelocityRel(0.25));
		ScraperMedium.move(lin(myFrame).setCartVelocity(70));
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		showTools app = new showTools();
		app.runApplication();
	}
}
