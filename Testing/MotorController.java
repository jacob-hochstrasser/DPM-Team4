package ca.mcgill.ecse211.testing;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class MotorController {
	//-----<Important Constant>-----//
	private static final double RADIUS = 1.58;//cm
	private static final double TRACK = 15;//cm
	volatile private static int ACCELERATION = 500;
	volatile private static int TEST_SPEED = 150;
	private static final int TIME_INTERVAL = 15;//ms
		
	//-----<Motor>-----//
	private static EV3LargeRegulatedMotor LEFT_MOTOR;
	private static EV3LargeRegulatedMotor RIGHT_MOTOR;
	
	//-----<Nested MotorController>-----//
	private static MotorController mc;
	
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
	
	public void setAcceleration(int newAcceleration) {
		ACCELERATION = newAcceleration;
		initializeAcceleration();
	}
	
	public void setSpeed(int newSpeed) {
		TEST_SPEED = newSpeed;
		initializeSpeed();
	}
	
	public void initializeSpeed() {
		synchronized(MotorController.class) {
			LEFT_MOTOR.setSpeed(TEST_SPEED);
			RIGHT_MOTOR.setSpeed(TEST_SPEED);
		}
	}
	
	public void initializeAcceleration() {
		synchronized(MotorController.class) {
			LEFT_MOTOR.setAcceleration(ACCELERATION);
			RIGHT_MOTOR.setAcceleration(ACCELERATION);
		}
	}
	
	public void moveBackword(double distance) {
		if(distance<0) {moveForward(distance);}
			
		synchronized(MotorController.class) {
			LEFT_MOTOR.rotate(-convertDistance(distance), true);
			RIGHT_MOTOR.rotate(-convertDistance(distance), false);
		}
			
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	public void moveForward(double distance) {
		if(distance<0) {moveBackword(distance);}
		
		synchronized(MotorController.class) {
			LEFT_MOTOR.rotate(convertDistance(distance), true);
			RIGHT_MOTOR.rotate(convertDistance(distance), false);
		}
		
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
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
