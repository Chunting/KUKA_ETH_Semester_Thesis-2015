package sandbox;

import info.monitorenter.gui.chart.Chart2D;
import info.monitorenter.gui.chart.IAxis;
import info.monitorenter.gui.chart.IAxisLabelFormatter;
import info.monitorenter.gui.chart.IPointPainter;
import info.monitorenter.gui.chart.ITrace2D;
import info.monitorenter.gui.chart.axis.AAxis;
import info.monitorenter.gui.chart.axis.AxisLinear;
import info.monitorenter.gui.chart.labelformatters.LabelFormatterDate;
import info.monitorenter.gui.chart.pointpainters.PointPainterDisc;
import info.monitorenter.gui.chart.rangepolicies.RangePolicyFixedViewport;
import info.monitorenter.gui.chart.traces.Trace2DLtd;
import info.monitorenter.gui.chart.traces.Trace2DSimple;
import info.monitorenter.gui.chart.traces.painters.TracePainterVerticalBar;
import info.monitorenter.gui.chart.views.ChartPanel;
import info.monitorenter.util.Range;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.text.SimpleDateFormat;
import java.util.Random;
import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.BorderFactory;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.UIManager;
import javax.swing.border.TitledBorder;

import com.sun.prism.BasicStroke;

import java.awt.Font;

public class Plot_StaticTrce {


	public static void main(String[] args) { 

		 // Generate empty data containers:
	    ITrace2D trace1 = new Trace2DLtd(200); 
	    trace1.setColor(Color.RED);
	    trace1.setName("trace 1");
	    trace1.setPhysicalUnits("1","sec");
	    
	    ITrace2D trace2 = new Trace2DSimple();
	    trace2.setColor(Color.BLUE);
	    trace2.setName("trace 2");
	    
	    // Create chart and connect:
	    Chart2D plot = new Chart2D();
	    plot.addTrace(trace1);
	    plot.addTrace(trace2);
	    
	    // Generate Data:
	    Random random = new Random();
	    for(int i=100;i>=0;i--){
	    	trace1.addPoint(i,random.nextDouble()*10.0+i);
	    	trace2.addPoint(i,random.nextDouble()*10.0+i/2);
	    }
		
	    // Make plot visible in frame:
	    JFrame frame = new JFrame("Plot of random process"); 
	    frame.getContentPane().add(plot);
	    frame.setSize(600,400);
	    // Enable the termination button [cross on the upper right edge]: 
	    frame.addWindowListener(new WindowAdapter(){
	    	public void windowClosing(WindowEvent e){
	          		System.exit(0);
	          		}
	    	});
	    frame.setVisible(true); 
	    
	
	    
	    
	    
	}

}
