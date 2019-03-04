package ca.mcgill.ecse211.testing;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class DrivingTest {
	private static final double TILE_SIZE = 30.48;//cm
	public static void main(String[] args) {
		MotorController mc = MotorController.getMotorController("B", "D");
		
		//Square Drive
		for(int i = 0; i<4; i++) {
			mc.moveForward(TILE_SIZE*3);
			mc.turnLeft(90);
		}
	}
	
	
}
