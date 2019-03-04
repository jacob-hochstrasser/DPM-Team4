package ca.mcgill.ecse211.testing;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class MotorController {
	
	/** 
	 * This is a class that controlls motors. Many of the movements could be find here.
	 * 
	 * @author Pengnan Fan
	 *
	 */
	
	//-----<Important Constant>-----//
	private static final double RADIUS = 1.58;//cm. It is certifed by testing from Pengnan.
	private static final double TRACK = 15;//cm. It is certified by testing from Pengnan.
	volatile private static int ACCELERATION = 500;
	volatile private static int TEST_SPEED = 150;
	private static final int TIME_INTERVAL = 15;//ms
		
	//-----<Motor>-----//
	private static EV3LargeRegulatedMotor LEFT_MOTOR;
	private static EV3LargeRegulatedMotor RIGHT_MOTOR;
	
	//-----<Nested MotorController>-----//
	private static MotorController mc;
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This is a static method used to get a MotorController object.
	 * Note: Only one of this type of object can be accessed.
	 * 
	 * @param leftMotor:String
	 * This input shows the port used for left motor.
	 * 
	 * @param rightMotor:String
	 * This input shows the port used for right motor.
	 * 
	 * @return mc:MotorController
	 * This output is the MotorController nested in the class.
	 * 
	 */
	public static MotorController getMotorController(String leftMotor, String rightMotor) {
		if(mc==null) {
			mc = new MotorController(leftMotor, rightMotor);
		}
		return mc;
	}
	
	private MotorController(String leftMotor, String rightMotor) {
		LEFT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort(leftMotor));
		RIGHT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort(rightMotor));
		initializeAcceleration();
		initializeSpeed();
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method reset the acceleration and re-initialize the motor.
	 * 
	 * @param newAcceleration:int
	 * This input is the new acceleration.
	 * 
	 */
	public void setAcceleration(int newAcceleration) {
		ACCELERATION = newAcceleration;
		initializeAcceleration();
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method reset the speed and re-initialize the motor.
	 * 
	 * @param newSpeed:int
	 * This input is the new speed.
	 * 
	 */
	public void setSpeed(int newSpeed) {
		TEST_SPEED = newSpeed;
		initializeSpeed();
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method initialize the speed of motors.
	 * 
	 */
	public void initializeSpeed() {
		synchronized(MotorController.class) {
			LEFT_MOTOR.setSpeed(TEST_SPEED);
			RIGHT_MOTOR.setSpeed(TEST_SPEED);
		}
	}

	/**
	 * @author Pengnan Fan
	 * 
	 * This method initialize the acceleration of motors.
	 * 
	 */
	public void initializeAcceleration() {
		synchronized(MotorController.class) {
			LEFT_MOTOR.setAcceleration(ACCELERATION);
			RIGHT_MOTOR.setAcceleration(ACCELERATION);
		}
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method drives the robot backward to a given distance. It will automatically call moveForward if the distance is negative
	 * 
	 * @param distance:int
	 * This input shows how much the robot should move backward in centimeter.
	 * 
	 */
	public void moveBackward(double distance) {
		if(distance<0) {moveForward(distance);}
			
		synchronized(MotorController.class) {
			LEFT_MOTOR.rotate(-convertDistance(distance), true);
			RIGHT_MOTOR.rotate(-convertDistance(distance), false);
		}
			
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method drives the robot forward to a given distance. It will automatically call moveBackward if the distance is negative.
	 * 
	 * @param distance:int
	 * This input shows how much the robot should move forward in centimeter.
	 * 
	 */
	public void moveForward(double distance) {
		if(distance<0) {moveBackward(distance);}
		
		synchronized(MotorController.class) {
			LEFT_MOTOR.rotate(convertDistance(distance), true);
			RIGHT_MOTOR.rotate(convertDistance(distance), false);
		}
		
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method turns the robot clockwise to a given degree. It will automatically call turnRight if the input degree is negative.
	 * 
	 * @param dTheta
	 * This input shows how much the robot should turn clockwise in degrees.
	 * 
	 */
	public void turnLeft(double dTheta) {
		if (dTheta<0) {
			turnRight(-dTheta);
		} else {
			synchronized(MotorController.class) {
				LEFT_MOTOR.rotate(-convertAngle(dTheta), true);
			    RIGHT_MOTOR.rotate(convertAngle(dTheta), false);
			}
		}
		
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method turns the robot counter clockwise to a given degree. It will automatically call turnLeft if the input degree is negative.
	 * 
	 * @param dTheta
	 * This input shows how much the robot should turn counter clockwise in degrees.
	 * 
	 */
	public void turnRight(double dTheta) {
		if (dTheta<0) {
		    turnLeft(-dTheta);
		} else {
			synchronized(MotorController.class) {
				LEFT_MOTOR.rotate(convertAngle(dTheta), true);
			    RIGHT_MOTOR.rotate(-convertAngle(dTheta), false);
			}
		}
		
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method will stop the robot immediately.
	 */
	public void stop() {
		synchronized(MotorController.class) {
			LEFT_MOTOR.stop(true);
			RIGHT_MOTOR.stop(false);
		}
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	private int convertAngle(double angle) {return convertDistance(Math.PI * TRACK * angle / 360.0);}
	  
	private int convertDistance(double distance) {return (int) ((180.0 * distance) / (Math.PI * RADIUS));}
}
