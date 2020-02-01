//import javax.swing.JOptionPane;
package frc.robot;

import java.lang.Math;
//import edu.wpi.first.networktables.NetworkTableEntry;
public class LimelightMath {

	public static void PrintLimeLightVals(double tx, double ty, double ta)
	{	
	    double x = tx; 
	    //the distance between the robot and the power cube (needs to be imported) 
	    double y = ty; 
	    //the height of the limelight camera above the cube (once known we can input that straight into the math)
	    double v = ta; 
	    //(v = dx) the velocity/change in position of the robot over a period of time (needs to be imported)
	    double h = calcLimelightDistance(x, y); 
	    //the calculated distance between the limelight and the power cube
	    double dh = changeInLimelightDistance(x,y,v); 
	    //the calculated change in distance between the limelight and the power cube
	    double o1 = Math.toDegrees(angleOfElevation(x,y)); 
	    //the calculated measure of the angle of elevation of the limelight
	    double o2 = Math.toDegrees(angleOther(x,y)); 
	    //the calculated measure of the complement to the angle of elevation 
	    double do1 = Math.toDegrees(-1 * changeInAngle(x,y)); 
	    //the calculated change in the angle of elevation 
	    double do2 = Math.toDegrees(changeInAngle(x,y)); 
	    //the calculated change of the complement to the angle of elevation 
	    if (v == 0)
	    {
	    		do1 = 0;
	    		do2 = 0;
	    }
	    System.out.println("Distance = " + h + " in");
	    System.out.println("Change in Diagonal Distance = " + dh + " in/s");
	    System.out.println("Angle of Elevation = " + o1 + " degrees");
	    System.out.println("Other Angle = " + o2 + " degrees");
	    System.out.println("Change in Angle of Elevation = " + do1 + " degrees/s");
	    System.out.println("Change in Other Angle = " + do2 + " degrees/s");
	}
	public static double calcLimelightDistance(double a, double b)
	{
		return Math.hypot(a, b); 
		//method to calculate h; a = x & b = y
	}
	public static double changeInLimelightDistance(double a, double b, double c)
	{
		return (a * c) / Math.hypot(a, b); 
		//method to calculate dh; a = x, b = y & c = v
	}
	public static double angleOfElevation(double a, double b)
	{
		return Math.acos(a / (Math.hypot(a, b))); 
		//method to calculate o1; a = x & b = y
	}
	public static double angleOther(double a, double b)
	{
		return Math.asin(a / (Math.hypot(a, b))); 
		//method to calculate o2; a = x & b = y
	}
	public static double changeInAngle(double a, double b)
	{
		return 1 / (b * (1 + Math.pow(a, 2) / Math.pow(b, 2))); 
		//method to calculate do2 (do1 = -1 * do2); a = x & b = y
	}
}