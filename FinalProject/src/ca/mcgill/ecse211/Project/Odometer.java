/* This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.Project;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import java.lang.Math;

/**
 * This class implements the Odometer for ECSE211 Lab 4 on the 
 * Lego EV3 platform.
 * 
 *  @author Julie Bellia, Pengnan Fan
 *  @debug Pengnan Fan
 *  
 */

public class Odometer extends Thread {
  
  //------<Important Constant>------//
  private final double TRACK = Navigation.TRACK;
  private final double RADIUS = Navigation.RADIUS;
  private static final long ODOMETER_PERIOD = 10; // odometer update period in ms
  
  //------<Counter>------//
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  
  //------<Motors>------//
  private EV3LargeRegulatedMotor leftMotor = Navigation.LEFT_MOTOR;
  private EV3LargeRegulatedMotor rightMotor = Navigation.RIGHT_MOTOR;
  
  //------<Position>------//
  private static double[] position = new double[] {0,0,0};

  public Odometer() {}
  
  /**
   *  run method (required for Thread)
   */
  public void run() {
    long updateStart, updateEnd;
    
    int pastMotorTachoCountL = leftMotorTachoCount;
    int pastMotorTachoCountR = rightMotorTachoCount;
    while (true) {
      updateStart = System.currentTimeMillis();
      
      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();
      
      double distL, distR, deltaD, dT, dX, dY, newT;
      
      distL = 2 * Math.PI * RADIUS * (leftMotorTachoCount - pastMotorTachoCountL) / 360;
      distR = 2 * Math.PI * RADIUS * (rightMotorTachoCount - pastMotorTachoCountR) / 360;
      pastMotorTachoCountL = leftMotorTachoCount;
      pastMotorTachoCountR = rightMotorTachoCount;
      deltaD = 0.5 * (distL + distR);
      
      dT = Math.toDegrees(Math.atan((distL - distR)/ TRACK));//In degreee
      newT = ((position[2] + (360 + dT) % 360) % 360) * (Math.PI/180);
      dX = deltaD * Math.sin(newT);
      dY = deltaD * Math.cos(newT);
      
      position[0]+=dX;
      position[1]+=dY;
      position[2]+=dT;
      if(position[2]<0) {
    	position[2]+=360;
      }
      if(position[2]>=360) {
    	position[2]-=360;
      }
      
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
      
    }
  }
  
  /**
   * This method returns a double array of current positions
   * @return
   */
  public static double[] getPosition() {
	return position;
  }
  
  public static double getX() {return position[0];}
  
  public static double getY() {return position[1];}
  /**
   * This method returns the current angle data
   * @return
   */
  public static double getT() {return position[2];}
  
  /**
   * This method sets the horizontal position as x
   * @param x
   */
  public static void setX(double x) {position[0] = x;}
  
  /**
   * This method sets the vertical position as y
   * @param y
   */
  public static void setY(double y) {position[1] = y;}
  
  /**
   * This method sets the angular position as t 
   * @param t
   */
  public static void setT(double t) {position[2] = t;}
  
  /**
   * This method resets the angular position as 0
   */
  public static void resetTheta() {setT(0);}
}

