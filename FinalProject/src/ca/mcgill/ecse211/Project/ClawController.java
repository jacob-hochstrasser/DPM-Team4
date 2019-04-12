package ca.mcgill.ecse211.Project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ClawController {
	private static EV3LargeRegulatedMotor CLAW;
	
	public ClawController(String clawPort) {
		CLAW = new EV3LargeRegulatedMotor(LocalEV3.get().getPort(clawPort));
		CLAW.setSpeed(500);
		CLAW.setAcceleration(500);
	}
	
	public void open() {
		CLAW.rotate(-90);
	}
	
	public void close() {
		CLAW.rotate(90);
	}
	
	public void demo(){
		while(true) {
    		close();
    		try {Thread.sleep(5000);} catch (InterruptedException e) {}
    		open();
    		try {Thread.sleep(5000);} catch (InterruptedException e) {}
    	}
	}
	
}
