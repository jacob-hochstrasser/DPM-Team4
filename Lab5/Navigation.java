package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
	
	//-----<Important Constants>-----//
	private final double LS_DIFF = 9.6;//cm
	private final double LS_WHEEL_DIFF = 12;//cm
	private final int INITIALIZING_SCOPE = 100;
	private final int MEASURING_SCOPE = 5;
	private final int TURNING_SPEED = 100;
	private final int LOCALIZING_SPEED = 75;
	private final int NAVIGATING_SPEED = 200;
	private float BLACK_LINE_LEFT;
	private float BLACK_LINE_RIGHT;
	private int START_CORNER;// 0 -> down left, 1 -> down right, 2 -> up right, 3 -> up left
	
	//-----<Motors>-----//
	private static EV3LargeRegulatedMotor LEFT_MOTOR;
	private static EV3LargeRegulatedMotor RIGHT_MOTOR;
	
	//-----<SensorPoller>-----//
	private static SensorPoller LOCALIZING_LEFT;
	private static SensorPoller LOCALIZING_RIGHT;
	
	
	public Navigation(EV3LargeRegulatedMotor LEFT_MOTOR, EV3LargeRegulatedMotor RIGHT_MOTOR, SensorPoller LOCALIZING_LEFT, SensorPoller LOCALIZING_RIGHT, int START_CORNER) {
		this.LEFT_MOTOR = LEFT_MOTOR;
		this.RIGHT_MOTOR = RIGHT_MOTOR;
		this.LOCALIZING_LEFT = LOCALIZING_LEFT;
		this.LOCALIZING_RIGHT = LOCALIZING_RIGHT;
		this.START_CORNER = START_CORNER;
		Sound.setVolume(100);
	}
	
	public void initialize() {
		float sum_left = 0;
		float sum_right = 0;
		for(int i = 0; i<INITIALIZING_SCOPE; i++) {
			sum_left+=LOCALIZING_LEFT.getData();
			sum_right+=LOCALIZING_RIGHT.getData();
		}
		BLACK_LINE_LEFT = sum_left/INITIALIZING_SCOPE;
		BLACK_LINE_RIGHT = sum_right/INITIALIZING_SCOPE;
	}
	
	public void localize() {
		if (START_CORNER == 0) {
			
		} else if (START_CORNER == 1) {
			
		} else if (START_CORNER == 2) {
			
		} else if (START_CORNER == 3) {
			
		} else {
			Sound.beep();
			try {Thread.sleep(100);} catch (InterruptedException e) {}
			Sound.beep();
			try {Thread.sleep(100);} catch (InterruptedException e) {}
			Sound.beep();
			System.exit(0);
		}
	}
	
	public boolean[] detectLine() {
		boolean[] result = new boolean[3];
		float leftData = LOCALIZING_LEFT.getData(MEASURING_SCOPE);
		float rightData = LOCALIZING_RIGHT.getData(MEASURING_SCOPE);
		float lRate = leftData/BLACK_LINE_LEFT;
		float rRate = rightData/BLACK_LINE_RIGHT;
		result[0] = 1.25 < lRate || 0.75 > lRate;
		result[1] = 1.25 < rRate || 0.75 > rRate;
		result[2] = result[0] || result[1];
		return result;
	}
	
	
	public void demo() {
		initialize();
		//LEFT_MOTOR.setSpeed(LOCALIZING_SPEED);
		//RIGHT_MOTOR.setSpeed(LOCALIZING_SPEED);
		//try {Thread.sleep(100);} catch (InterruptedException e) {}
		//LEFT_MOTOR.forward();
		//RIGHT_MOTOR.forward();
		do {
			float leftData = LOCALIZING_LEFT.getData(MEASURING_SCOPE);
			float rightData = LOCALIZING_RIGHT.getData(MEASURING_SCOPE);
			float lRate = leftData/BLACK_LINE_LEFT;
			float rRate = rightData/BLACK_LINE_RIGHT;
			boolean result1 = 1.15 < lRate || 0.85 > lRate;
			boolean result2 = 1.15 < rRate || 0.85 > rRate;
			System.out.println("STD = " + BLACK_LINE_LEFT + "\nRatio = " + lRate);
			if(result1) {
				Sound.beep();
				//LEFT_MOTOR.stop();
			}
			if(result2) {
				Sound.twoBeeps();
				//RIGHT_MOTOR.stop();
				//break;
			}
		} while(true);
		/*
		initialize();
		LEFT_MOTOR.setSpeed(NAVIGATING_SPEED);
		RIGHT_MOTOR.setSpeed(NAVIGATING_SPEED);
		boolean[] detection = detectLine();
		do {
		LEFT_MOTOR.forward();
		RIGHT_MOTOR.forward();
		detection = detectLine();
		} while(!detection[2]);
		LEFT_MOTOR.stop(true);
		RIGHT_MOTOR.stop(false);
		*/
	}
}
