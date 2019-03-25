package ca.mcgill.ecse211.Project;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * This is the testing class for block test.
 * @author Pengnan Fan
 *
 */

public class TestClass {
	
	/**
     * String identifying the port of the left motor
     */
    private static final String LEFT_MOTOR = "B";

    /**
     * String identifying the port of the right motor
     */
    private static final String RIGHT_MOTOR = "D";
	
	/**
     * The navigation class that handles all methods for localizing, moving, and orienting the robot
     */
    private static final Navigation NAV = Navigation.getNavigation(LEFT_MOTOR, RIGHT_MOTOR);
	
    /**
     * The odometer that keeps track of the robot's position and heading
     */
    private static final Odometer ODO = new Odometer();
    
    /**
     * The motor used to scan around cans.
     */
    private static final EV3MediumRegulatedMotor SCANNER = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

    /**
     * The Sample Provider for the identifying color sensor
     */
    private static final SampleProvider IDRGB = new EV3ColorSensor(LocalEV3.get().getPort("S4")).getRGBMode();

    /**
     * The text LCD of the containing EV3 brick
     */
    static final TextLCD LCD = LocalEV3.get().getTextLCD();
    
    /**
     * targetColor
     * {
     * 		1		->	blue
     * 		2		->	green
     * 		3		->	yellow
     * 		4		->	red
     * 		default	->	any
     * }
     */
    private static final int targetColor = 1;
    
    private static Identifier ID = new Identifier(SCANNER, targetColor, IDRGB, 1000, LCD);
    
    private static final int[] LL = new int[] {0, 0};
    private static final int[] UR = new int[] {2, 2};
    private static final int START_CORNER = 0;
    
    private static final double TILE_SIZE = 30.28;
    
    
	public static void main(String[] args) {
		ODO.start();
		NAV.setup(LL, UR, START_CORNER);
		/* Add testing method below */
		
		
		
		/* Add testing method above */
	}
	
	/**
	 * This method will proceed a localization
	 */
	private static void localizationTest() {
		NAV.localize();
	}
	
	/**
	 * 	It will beep once if selected color is found, or twice if it is not found.
	 */
	private static void colorIdentificationTest() {
		boolean result = ID.isTargetCan();
		if (result) {
			Sound.beep();
		} else {
			Sound.twoBeeps();
		}
	}
	
}
