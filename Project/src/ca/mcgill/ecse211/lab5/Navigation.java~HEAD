package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class implements the navigation part for ECSE211 Lab 3 on the 
 * Lego EV3 platform.
 * 
 *  @author Pengnan Fan
 *  @debug Pengnan Fan
 *  
 */

public class Navigation {
  
  //------<Important Constant>------//
  private static final int LEFT_SPEED = 150;
  private static final int RIGHT_SPEED = 150;
  private static final int ROTATE_SPEED = 50;
  private static final int HIGH_SPEED = 120;
  private static final int LOW_SPEED = 72;
  private static final int STOP_SPEED = 25;
  private static final double NAVI_ERROR = 0.5;//cm
  private static final double BACK_DISTANCE = 2.5;//cm
  private static int TIME_OUT = 1250;
  private static int TIME = 0;
  private static double RADIUS;//cm
  private static double TRACK;//cm
  private static double TILE_SIZE;//cm
  private static int MINIMUM;
  private static int MIN;
  private static int INFTY;
    
  //------<Motor>------///
  private static EV3LargeRegulatedMotor leftMotor;
  private static EV3LargeRegulatedMotor rightMotor;
  
  //------<Position>------//
  private static double[] pos = new double[] {0,0,0};
  
  //------<State>------//
  private static boolean isMoving  = false;
  private static boolean isAvoiding = false;
  
  /**
   * This is the constructor of the class
   * 
   * @param leftMotor. The left motor of the robot
   * @param right motor. The right motor of the robot
   * @param TILE_SIZE. This the length of the side of a tile in centimeter
   * @param MINMUM. This is the minimum distance between the robot and the wall in centimeter
   * @param MIN. This is the distance expected to keep when avoding the wall in centimeter
   * @param INFTY. This is the maximum distance between the robot and the wall in centimeter
   * @param RADIUS. This is the radius of a wheel in centimeter
   * @param TRACK. This is the distance between two wheels in centimeter
   */
  
  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TILE_SIZE, int MINIMUM, int MIN, int INFTY, double RADIUS, double TRACK) {
    this.MINIMUM = MINIMUM;
    this.MIN = MIN;
    this.INFTY = INFTY;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.RADIUS = RADIUS;
    this.TRACK = TRACK;
    this.TILE_SIZE = TILE_SIZE;
  }
  
  /**
   * This method update the position of the robot.
   * @param position. The position of the robot whith 0 = x, 1 = y , 2 = theta in actual size (cm)
   */
  public static void updatePosition(double[] position) {pos = position;}
  
  /**
   * This method reset the isAvoiding state
   */
  public void resetAvoiding() {isAvoiding = false;}
  
  /**
   * This method return the state of isAvoiding
   */
  public boolean isAvoiding() {return isAvoiding;}
  
  /**
   * This method stop the robot.
   */
  public void stop() {
	leftMotor.setSpeed(STOP_SPEED);
	rightMotor.setSpeed(STOP_SPEED);
	leftMotor.backward();
	leftMotor.backward();
	leftMotor.stop();
	rightMotor.stop();
  }
  
  /**
   * This method check if the robot is arrived to the coordinate (x, y)
   * 
   *  @param x. The coordinate of x-axis in the unit of TILE_SIZE
   *  @param y. The coordinate of y-axis in the unit of TILE_SIZE
   */
  public boolean isArrived(double x, double y) {
	pos = Odometer.getPosition();
	return Math.hypot((x*TILE_SIZE)-pos[0], (y*TILE_SIZE)-pos[1]) <= NAVI_ERROR;
  }
  
  /**
   * This method allows obstacle avoiding during normal travelling to a corrdinate
   * 
   * @param x. The coordinate of x-axis in the unit of TILE_SIZE
   * @param y. The coordinate of y-axis in the unit of TILE_SIZE
   * @param obstacle. True -> found obstacle, False -> no obstacle
   * @param distance. distance read by the ultrasonic sensor in centimeter
   */
  public void advancedTravelTo(double x, double y, boolean obstacle, int distance) {
	if(!obstacle&&!isAvoiding) {
	  //No obstacle
      if(isMoving) {
	    if(isArrived(x, y)) {
		  stop();
		  try {
			Thread.sleep(250);
		  } catch (InterruptedException e) {}
		  SensorController.counter++;
		  isMoving = false;
		}
		return;
	  } else {
		//Normal
		Sound.beep();
		isMoving = true;
		isAvoiding = false;
		stop();
		try {
		  Thread.sleep(250);
		} catch (InterruptedException e) {}
		pos = Odometer.getPosition();
	    double dX = x*TILE_SIZE - pos[0];
		double dY = y*TILE_SIZE - pos[1];
	    double t = calculateTheta(dX, dY);
		//System.out.println("Theta = " + t);
		turnTo(t);
		leftMotor.setSpeed(LEFT_SPEED);
		rightMotor.setSpeed(RIGHT_SPEED);
		leftMotor.forward();
		rightMotor.forward();    
	  }
	} else {
	  //There is an obstacle
	  if(!isAvoiding) {
		isAvoiding = true;
		isMoving = false;
		TIME = 0;
		stop();
		leftMotor.setSpeed(LEFT_SPEED);
		rightMotor.setSpeed(RIGHT_SPEED);
		leftMotor.rotate(convertDistance(RADIUS, -BACK_DISTANCE), true);
	    rightMotor.rotate(convertDistance(RADIUS, -BACK_DISTANCE), false);
		rightMotor.rotate(-convertAngle(RADIUS, TRACK, 90), true);
		leftMotor.rotate(convertAngle(RADIUS, TRACK, 90), false);
		/*
		leftMotor.rotate(convertDistance(RADIUS, 30), true);
	    rightMotor.rotate(convertDistance(RADIUS, 30), false);
	    rightMotor.rotate(-convertAngle(RADIUS, TRACK, -90), true);
		leftMotor.rotate(convertAngle(RADIUS, TRACK, -90), false);
		leftMotor.rotate(convertDistance(RADIUS, 30), true);
	    rightMotor.rotate(convertDistance(RADIUS, 30), false);
	    */
	    stop();
	  } else {
		//Avoiding
	    if(distance>=INFTY) {
	      leftMotor.setSpeed(LOW_SPEED);
		  rightMotor.setSpeed(HIGH_SPEED);
		  leftMotor.forward();
		  rightMotor.forward();
	    } else if (distance>=0.95*MIN&&distance<=1.05*MIN) {   
		  leftMotor.setSpeed(HIGH_SPEED);
		  rightMotor.setSpeed(HIGH_SPEED);
		  leftMotor.forward();
		  rightMotor.forward();
		} else if (distance > 1.05*MIN) {
		  leftMotor.setSpeed(LOW_SPEED);
		  rightMotor.setSpeed(HIGH_SPEED);
		  leftMotor.forward();
		  rightMotor.forward();
		} else if (distance < 0.95*MIN){
		  leftMotor.setSpeed(HIGH_SPEED);
		  rightMotor.setSpeed(LOW_SPEED);
		  leftMotor.forward();
		  rightMotor.forward();
		}
		isAvoiding = ++TIME <= TIME_OUT;
		System.out.println("TIME = " + TIME);
		if(!isAvoiding) {
		  stop();
		}
	  }
	  
	}
  }
  
  private double calculateTheta(double dX, double dY) {
	double headingTheta = 0;
	if(dX>0) {
	  if(dY>0) {
		headingTheta = Math.toDegrees(Math.atan(dX/dY));
	  } else if(dY<0) {
		headingTheta = (Math.toDegrees(Math.atan(dX/dY))+180)%180;
	  } else {
		headingTheta = 90;
	  }
	} else if (dX<0){
	  if(dY>0) {
		headingTheta = (Math.toDegrees(Math.atan(dX/dY))+360)%360;
	  } else if(dY<0) {
		headingTheta = Math.toDegrees(Math.atan(dX/dY))+180;
	  } else {
		headingTheta = 270;
	  }
	} else {
	  if(dY>0) {
		headingTheta = 0;
	  } else if(dX<0) {
		headingTheta = 180;
	  } else {
		headingTheta = 0;
	  }
	}
	return headingTheta;
  }
  
  /**
   * This method is a simple version of travelling to a given point (x, y). It cannot
   * avoid any obstacles.
   * 
   * @param x. The coordinate of x-axis in the unit of TILE_SIZE
   * @param y. The coordinate of y-axis in the unit of TILE_SIZE
   */ 
  public void travelTo(double x, double y) {
	isMoving = true;
	x*=TILE_SIZE;
	y*=TILE_SIZE;
	double dX = x - pos[0];
	double dY = y - pos[1];
	
	turnTo(calculateTheta(dX, dY));
	
	double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
	
	leftMotor.setSpeed(LEFT_SPEED);
    rightMotor.setSpeed(RIGHT_SPEED);
	leftMotor.rotate(convertDistance(RADIUS, distance), true);
    rightMotor.rotate(convertDistance(RADIUS, distance), false);

    isMoving = false;
    Sound.beep();
  }
  
	
  /**
   * This method causes the robot to turn (on point) to the absolute heading theta
   * 
   * @param Theta. The angle to turn to
   */
  public void turnTo(double Theta) {// Theta is in the range of [0, 359.9]
    pos = Odometer.getPosition();
	double dTheta = pos[2] - Theta;
	leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    if(dTheta<-180) {
      dTheta+=360;
    } else if(dTheta>180) {
      dTheta-=360;
    }
    
    rightMotor.rotate(convertAngle(RADIUS, TRACK, dTheta), true);
    leftMotor.rotate(-convertAngle(RADIUS, TRACK, dTheta), false);
    
    leftMotor.stop();
	rightMotor.stop();
	try {
	  Thread.sleep(250);
	} catch (InterruptedException e) {}
  }
  
  /**
   * This method return the state of isNavigating
   */
  public boolean isNavigating() {return isMoving;}
  
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  
  private static int convertDistance(double radius, double distance) {
	return (int) ((180.0 * distance) / (Math.PI * radius));
  }
}
