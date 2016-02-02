package sandbox;

import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

public class Testing {

	public Testing() {
		// TODO Automatisch generierter Konstruktorstub
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Automatisch generierter Methodenstub

		System.out.println("Hello");
		
		
		// define frames for task trajectory:
		Frame F_root = new Frame(0,0,0);
		Frame F_contactEnd = new Frame(400,0,230,Math.toRadians(-144),Math.toRadians(15.5),Math.toRadians(-166));

		Frame F_contactStart;
		Frame F_flyMin;
		Frame F_flyMax;
		Frame F_fly3;
		Frame F_fly4;
		
		System.out.println(String.format("F_contactEnd, x: %1$f y: %2$f z: %3$f", F_contactEnd.getX(), F_contactEnd.getY(), F_contactEnd.getZ()));
		
		Frame F_tmp = F_contactEnd.copy();
				
		F_flyMin 		= F_tmp.transform(F_root,XyzAbcTransformation.ofTranslation(-150,0, 70)).copy();
		F_fly3 			= F_tmp.transform(F_root,XyzAbcTransformation.ofTranslation( 100,0, 40)).copy();
		F_fly4 			= F_tmp.transform(F_root,XyzAbcTransformation.ofTranslation( 150,0,  0)).copy();
		F_flyMax 		= F_tmp.transform(F_root,XyzAbcTransformation.ofTranslation( 100,0,-40)).copy();		
		F_contactStart 	= F_tmp.transform(F_root,XyzAbcTransformation.ofTranslation( -50,0,-70)).copy();
				
		System.out.println(String.format("F_flyMin, x: %1$f y: %2$f z: %3$f", F_flyMin.getX(), F_flyMin.getY(), F_flyMin.getZ()));
		System.out.println(String.format("F_fly3, x: %1$f y: %2$f z: %3$f", F_fly3.getX(), F_fly3.getY(), F_fly3.getZ()));
		System.out.println(String.format("F_fly4, x: %1$f y: %2$f z: %3$f", F_fly4.getX(), F_fly4.getY(), F_fly4.getZ()));
		System.out.println(String.format("F_flyMax, x: %1$f y: %2$f z: %3$f", F_flyMax.getX(), F_flyMax.getY(), F_flyMax.getZ()));
		System.out.println(String.format("F_contactStart, x: %1$f y: %2$f z: %3$f", F_contactStart.getX(), F_contactStart.getY(), F_contactStart.getZ()));
		
		Transformation trafo;
		
		trafo = F_flyMin.getTransformationFromParent();
		
		System.out.println(String.format("Trafo to F_flyMin: %f, %f, %f", trafo.getX(), trafo.getY(), trafo.getZ())); 
		System.out.println(String.format("integer: %d",5));
		
		TestingSub myTest = new TestingSub();
		
		myTest.showMe();
	}

}
