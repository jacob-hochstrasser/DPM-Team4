package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * This class implement the ultrasonic localizer for finding the angular 
 * orientation of the robot using the ultrasonic sensor
 * 
 * @author Pengnan Fan
 *
 */
public class UltrasonicLocalizer {
  
  //------<Important Constant>------//
  private int LOCALIZE_MODE;//0 -> rising edge, 1 -> falling edge
  private int SCANNING_SPEED;
  private int ACCELERATION;
  private static double TRACK;//cm
  private static double RADIUS;//cm
  private final double ROTATION_ERROR_CW = 20;//degree, error of clockwise rotation
  private final double ROTATION_ERROR_CCW = 11;//degree, error of counterclockwise rotation
  private final double TILE_SIZE = 30.48;//cm, length of one side of a tile
  private final double STANDARD_DISTANCE_FALLING = TILE_SIZE;//cm, standard distance to distinguish falling edge
  private final double STANDARD_DISTANCE_RISING = TILE_SIZE;//cm, standard distance to distinguish rising edge
  private final double NOTICE_MARGIN = 1;//cm
  private final int FILTER_SCOPE = 10;
  private final int PERIOD = 10;
  private final int INFTY = 255;//cm
  
  //------<Sensors>------//
  private static SampleProvider uData = 
    (new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"))).getMode("Distance");
  private static float[] distanceData;
  
  //------<Motors>------//
  private static EV3LargeRegulatedMotor LEFT_MOTOR;
  private static EV3LargeRegulatedMotor RIGHT_MOTOR;
  
  //------<Distance>------//
  private static float d1 = 0;
  private static float d2 = 0;
  
  /**
   * This is the constructor of this class.
   * 
   * @param mode:int - 0 for rising edge, 1 for falling edge
   * @param LEFT_MOTOR:EV3LargeRegulatedMotor - the left motor
   * @param RIGHT_MOTOR:EV3LargeRegulatedMotor - the right motor
   * @param SCANNING_SPEED:int - the speed for scanning
   */
  public UltrasonicLocalizer(int mode, EV3LargeRegulatedMotor LEFT_MOTOR, EV3LargeRegulatedMotor RIGHT_MOTOR, int SCANNING_SPEED, int ACCELERATION, double RADIUS, double TRACK) {
    LOCALIZE_MODE = mode;
    this.LEFT_MOTOR = LEFT_MOTOR;
    this.RIGHT_MOTOR = RIGHT_MOTOR;
    this.SCANNING_SPEED = SCANNING_SPEED;
    this.ACCELERATION = ACCELERATION;
    this.RADIUS = RADIUS;
    this.TRACK = TRACK;
    distanceData = new float[uData.sampleSize()];
  }
  
  /**
   * This method starts localizing angle by rising/falling edge based on mode
   */
  public void localizeAngle() {
	
	if (LOCALIZE_MODE == 0) {
	  this.risingEdge();
	} else if (LOCALIZE_MODE == 1) {
	  this.fallingEdge();
	}
  }
  
  /**
   * This method proceeds falling edge localization
   */
  private void fallingEdge() {
	// Set speed
	LEFT_MOTOR.setSpeed(SCANNING_SPEED);
	RIGHT_MOTOR.setSpeed(SCANNING_SPEED);
	LEFT_MOTOR.setAcceleration(ACCELERATION);
	RIGHT_MOTOR.setAcceleration(ACCELERATION);
	
	try {Thread.sleep(250);} catch (InterruptedException e) {}
	
	double[] pos1 = new double[] {0,0,0};
	boolean isDetected = false;
	d1 = measurementWithFilter();
	d2 = 0;
	
	// Rotate clockwise until you detect a falling edge
    while(!isDetected) {
      try {Thread.sleep(PERIOD);} catch (InterruptedException e) {}
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.backward();
      d2 = d1;
      d1 = measurementWithFilter();
      if((d2 >= (STANDARD_DISTANCE_FALLING + NOTICE_MARGIN))&&(d1<= (STANDARD_DISTANCE_FALLING - NOTICE_MARGIN))) {
    	stop();
    	isDetected = true;
    	pos1 = Odometer.getPosition();
      }
    }
    Sound.beep();
    
    double[] pos2 = new double[] {0,0,0};
    isDetected = false;
    
    d1 = measurementWithFilter();
	d2 = 0;
	
    // Rotate counter-clockwise until you detect another falling edge
    while(!isDetected) {
      try {Thread.sleep(PERIOD);} catch (InterruptedException e) {}
      LEFT_MOTOR.backward();
      RIGHT_MOTOR.forward();
      d2 = d1;
      d1 = measurementWithFilter();
      if((d2 >= (STANDARD_DISTANCE_FALLING + NOTICE_MARGIN))&&(d1<= (STANDARD_DISTANCE_FALLING - NOTICE_MARGIN))) {
      	stop();
      	isDetected = true;
      	pos2 = Odometer.getPosition();
      }
    }
    
    Sound.beep();
    stop();
    
    // Calculate angle of local maximum based on two detected edges and use it to find 0° 
    double dTheta = (-45 + (pos1[2]+pos2[2])/2 + 360)%360;
    try {Thread.sleep(500);} catch (InterruptedException e1) {}
    
    //Turn to 0
    turnTo(dTheta);
    turnLeft(ROTATION_ERROR_CW);//Fix rotational error
    Odometer.resetTheta();//Reset theta
  }
  
  /**
   * This method proceeds rising edge localization
   */
  private void risingEdge() {
    // Set speed
	LEFT_MOTOR.setSpeed(SCANNING_SPEED);
	RIGHT_MOTOR.setSpeed(SCANNING_SPEED);
	LEFT_MOTOR.setAcceleration(ACCELERATION);
	RIGHT_MOTOR.setAcceleration(ACCELERATION);
	
	try {Thread.sleep(250);} catch (InterruptedException e) {}
	
	double[] pos1 = new double[] {0,0,0};
	boolean isDetected = false;
	d1 = measurementWithFilter();
	d2 = 0;
	
	// Rotate clockwise until you detect a rising edge
	while(!isDetected) {
	  LEFT_MOTOR.forward();
	  RIGHT_MOTOR.backward();
	  d2 = d1;
      d1 = measurementWithFilter();
	  if((d1 >= (STANDARD_DISTANCE_RISING + NOTICE_MARGIN))&&(d2 <= (STANDARD_DISTANCE_RISING - NOTICE_MARGIN))) {
	  	stop();
	    isDetected = true;
	   	pos1 = Odometer.getPosition();
	  }
	}
	Sound.beep();
	
	double[] pos2 = new double[] {0,0,0};
    isDetected = false;
    LEFT_MOTOR.backward();
    RIGHT_MOTOR.forward();
    
    try {Thread.sleep(1000);} catch (InterruptedException e) {}
    
    // Rotate counter-clockwise until you detect another rising edge 
    while(!isDetected) {
      LEFT_MOTOR.backward();
      RIGHT_MOTOR.forward();
      d2 = d1;
      d1 = measurementWithFilter();
      if((d1 >= (STANDARD_DISTANCE_RISING + NOTICE_MARGIN))&&(d2 <= (STANDARD_DISTANCE_RISING - NOTICE_MARGIN))) {
        stop();
        isDetected = true;
        pos2 = Odometer.getPosition();
        }
    }
      
    Sound.beep();
    stop();
    
    // Calculate angle of local maximum based on two detected edges and use it to find 0°
    double dTheta = (-225 + (pos1[2]+pos2[2])/2 + 360)%360;
    try {Thread.sleep(500);} catch (InterruptedException e1) {}
    //Turn to 0
    turnTo(dTheta);
    turnRight(ROTATION_ERROR_CCW);//fix rotational error
    Odometer.resetTheta();//reset theta
  }
  
  /**
   * This method stops the robot immediately
   */
  private void stop() {
	LEFT_MOTOR.stop(true);
	RIGHT_MOTOR.stop(false);
  }
  
  /**
   * This method turns the robot to any given theta [0,359.9]
   */
  private void turnTo(double Theta) {// Theta is in the range of [0, 359.9]
	stop();
	double[] pos = Odometer.getPosition();
	double dTheta = pos[2] - Theta;
	LEFT_MOTOR.setSpeed(SCANNING_SPEED);
	RIGHT_MOTOR.setSpeed(SCANNING_SPEED);
	LEFT_MOTOR.setAcceleration(ACCELERATION);
	RIGHT_MOTOR.setAcceleration(ACCELERATION);
	   
	if(dTheta<-180) {
	  dTheta+=360;
	} else if(dTheta>180) {
	  dTheta-=360;
    }
	    
	LEFT_MOTOR.rotate(convertAngle(dTheta), true);
	RIGHT_MOTOR.rotate(-convertAngle(dTheta), false);
	    
	stop();
	try {Thread.sleep(250);} catch (InterruptedException e) {}
  }
  
  /**
   * This method turns the robot clockwise by a given degree
   * @param dTheta
   */
  private void turnLeft(double dTheta) {
	stop();
	LEFT_MOTOR.setSpeed(SCANNING_SPEED);
	RIGHT_MOTOR.setSpeed(SCANNING_SPEED);
	LEFT_MOTOR.setAcceleration(ACCELERATION);
	RIGHT_MOTOR.setAcceleration(ACCELERATION);
	
	try {Thread.sleep(100);} catch (InterruptedException e) {}
	
	if (dTheta<0) {
	  turnRight(-dTheta);
	} else {
	  LEFT_MOTOR.rotate(convertAngle(dTheta), true);
	  RIGHT_MOTOR.rotate(-convertAngle(dTheta), false);
	}
  }
  
  /**
   * This method turns the robot counter clockwise by a given degree
   * @param dTheta
   */
  private void turnRight(double dTheta) {
	stop();
	LEFT_MOTOR.setSpeed(SCANNING_SPEED);
	RIGHT_MOTOR.setSpeed(SCANNING_SPEED);
	LEFT_MOTOR.setAcceleration(ACCELERATION);
	RIGHT_MOTOR.setAcceleration(ACCELERATION);
	
	try {Thread.sleep(100);} catch (InterruptedException e) {}
	
	if (dTheta<0) {
	  turnLeft(-dTheta);
	} else {
	  LEFT_MOTOR.rotate(-convertAngle(dTheta), true);
	  RIGHT_MOTOR.rotate(convertAngle(dTheta), false);
	}
  }
  
  /**
   * This method provide a filtered measurement of the us sensor
   * @return
   */
  private float measurementWithFilter() {
	float output = 0;
	for(int i = 0; i<FILTER_SCOPE; i++) {
	  uData.fetchSample(distanceData, 0);
	  output = distanceData[0]*100;
	  if(output<=INFTY) {return output;}
	}
	return INFTY;
  }
  
  /**
   * This method returns the us data
   * @return
   */
  public static float getDistance() {return d1;}
  
  /**
   * This method converts an angle in degree to number of rotation of motors
   * @param angle
   * @return
   */
  private static int convertAngle(double angle) {return convertDistance(Math.PI * TRACK * angle / 360.0);}
  
  /**
   * This method converts a distance in cm to number of rotation of motors
   * @param distance
   * @return
   */
  private static int convertDistance(double distance) {return (int) ((180.0 * distance) / (Math.PI * RADIUS));}
  
}
