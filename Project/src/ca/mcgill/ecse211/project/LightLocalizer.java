package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class implement the light localization of ECSE211 lab 4
 * @author Julie Bellia and Pengnan Fan
 *
 */
public class LightLocalizer {
  
  private static final int ACCELERATION = 500;
  private static final int ROTATE_SPEED = 50;
  private static final int LEFT_SPEED = 150;
  private static final int RIGHT_SPEED = 150;
  private static final int SAMPLE_NUM = 5;
  private static final long SAMPLE_PERIOD = 200;
  private static final int LINE_FILTER = 1000;
  private static final double LSRADIUS = 12.2;
  private static final double DIS = LSRADIUS;
  private static double RADIUS;
  private static double TRACK;
  
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private static Port lsPort = LocalEV3.get().getPort("S4");
  private static SampleProvider light;
  private static SensorModes lightSensor;
  private float[] lightData;
  
  private static float STANDARD;
  
  /**
   * This constructor initializes an instance of LightLocalizer
   * 
   * @param left The left motor
   * @param right The right motor
   * @param radius The radius of the wheels
   * @param track The radius of rotation
   */
  public LightLocalizer(EV3LargeRegulatedMotor left, EV3LargeRegulatedMotor right, double radius, double track) {
    this.leftMotor = left;
    this.rightMotor = right;
    RADIUS = radius;
    TRACK = track;
    lightSensor = new EV3ColorSensor(lsPort);
    light = ((EV3ColorSensor)lightSensor).getRedMode();
    Sound.setVolume(100);
    this.lightData = new float[lightSensor.sampleSize()];
  }
  
  /**
   * This method moves the robot to the starting position
   */
  private void initialize() {
	standardization();
	
	try {Thread.sleep(250);} catch (InterruptedException e) {}
	
	leftMotor.setAcceleration(ACCELERATION);
	rightMotor.setAcceleration(ACCELERATION);
	leftMotor.setSpeed(ROTATE_SPEED);
	rightMotor.setSpeed(ROTATE_SPEED);
	
	try {Thread.sleep(250);} catch (InterruptedException e) {}
	
	leftMotor.rotate(convertDistance(RADIUS, DIS),true);
	rightMotor.rotate(convertDistance(RADIUS, DIS),false);
	
	try {Thread.sleep(250);} catch (InterruptedException e) {}
	
	turnTo(90);
	
	try {Thread.sleep(250);} catch (InterruptedException e) {}
	
	rightMotor.rotate(convertDistance(RADIUS, DIS),true);
	leftMotor.rotate(convertDistance(RADIUS, DIS),false);
	
	try {Thread.sleep(250);} catch (InterruptedException e) {}
	
	stop();
	
	try {Thread.sleep(250);} catch (InterruptedException e) {}
	
	turnTo(0);
	
  }
  
  /**
   * This method turns the robot to any given direction Theta [0, 359.9]
   * @param Theta
   */
  private void turnTo(double Theta) {// Theta is in the range of [0, 359.9]
	stop();
	double[] pos = Odometer.getPosition();
	double dTheta = pos[2] - Theta;
	leftMotor.setSpeed(ROTATE_SPEED);
	rightMotor.setSpeed(ROTATE_SPEED);
	leftMotor.setAcceleration(ACCELERATION);
	rightMotor.setAcceleration(ACCELERATION);
		   
	if(dTheta<-180) {
	  dTheta+=360;
	} else if(dTheta>180) {
	  dTheta-=360;
	}
		    
	leftMotor.rotate(-convertAngle(RADIUS, TRACK, dTheta), true);
	rightMotor.rotate(convertAngle(RADIUS, TRACK, dTheta), false);
		    
	stop();
	try {Thread.sleep(250);} catch (InterruptedException e) {}
  }
  
  /**
   * This method returns a boolean value suggesting the decection of a line
   * @return
   */
  private boolean isLineDetected() {
	float sumLight = 0;
	for(int i = 0; i < SAMPLE_NUM; i++) {
	  light.fetchSample(lightData, 0);
	  sumLight += lightData[0]*100;
	}
	sumLight/=SAMPLE_NUM;
	return sumLight/STANDARD > 1.1 || sumLight/STANDARD < 0.9;
  }
  
  /**
   * This method stops the robot immediately
   */
  private void stop() {
	leftMotor.stop(true);
	rightMotor.stop(false);
  }
  
  /**
   * This method helps to standardize the light data used as detection
   */
  private void standardization() {
	float sum = 0;
	for(int i = 0; i<100; i++) {
	  light.fetchSample(lightData, 0);
	  sum+=lightData[0]*100;
	}
	STANDARD = (sum)/100f;
  }
  
  /**
   * This method precisely locates the origin (0, 0) and drives to it
   * @author Pengnan Fan and Julie Bellia
   */
  public void findOrigin() {
	initialize();
	
	double[] lineTheta = new double[] {-1,-1,-1,-1};
	leftMotor.setSpeed(ROTATE_SPEED);
	rightMotor.setSpeed(ROTATE_SPEED);
	leftMotor.setAcceleration(ACCELERATION);
	rightMotor.setAcceleration(ACCELERATION);
	
	for(int i = 0; i<4; i++) {
	  while(!isLineDetected()) {
		leftMotor.forward();
		rightMotor.backward();
	  }
	  stop();
	  Sound.beep();
	  lineTheta[i] = Odometer.getT();
	  try {Thread.sleep(250);} catch (InterruptedException e) {}
	  leftMotor.forward();
	  rightMotor.backward();
	  try {Thread.sleep(500);} catch (InterruptedException e) {}
	}
	
	stop();
	
	try {Thread.sleep(250);} catch (InterruptedException e) {}
	turnTo(0);
	
	double thetaX = (lineTheta[2] - lineTheta[0]);
    double thetaY = (lineTheta[3] - lineTheta[1]);
    double x = LSRADIUS * Math.cos(Math.toRadians(thetaY / 2));
    double y = LSRADIUS * Math.cos(Math.toRadians(thetaX / 2));
    
    if (lineTheta[0] < 270.0 && lineTheta[2] > 90.0) { // 1st or 2nd quadrant
      if (lineTheta[1] < 359.9 && lineTheta[3] > 180.0) { // 1st quadrant
        // nothing happens, x and y are correct sign
    	x = -x;
      } else { // 2nd quadrant
        //x = -x;
      }
    } else { // 3rd or 4th quadrant
      if (lineTheta[1] < 359.9 && lineTheta[3] > 180.0) { // 4th quadrant
        y = -y;
        x = -x;
      } else { // 3rd quadrant
        
        y = -y;
      }
    }
    Odometer.setX(lineTheta[0]);
    Odometer.setY(lineTheta[2]);
    
    // Turn towards the origin
    
    double[] position = Odometer.getPosition();
    double dTheta = position[2] - calculateTheta(x, y);
    if (dTheta < -180) {
      dTheta += 360;
    } else if (dTheta > 180) {
      dTheta -= 360;
    }
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    rightMotor.rotate(-convertAngle(RADIUS, TRACK, dTheta), true);
    leftMotor.rotate(convertAngle(RADIUS, TRACK, dTheta), false);
    
    // Drive straight to the origin
    double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
	leftMotor.setSpeed(LEFT_SPEED);
    rightMotor.setSpeed(RIGHT_SPEED);
	leftMotor.rotate(convertDistance(RADIUS, distance), true);
    rightMotor.rotate(convertDistance(RADIUS, distance), false);
    
    // Turn to the 0°
    turnTo(0);
  }
  
  /**
   * This method precisely locates the origin (0, 0) and drives to it
   */
  public void localizeOrigin() {
	initialize();
    double sumLight = 0;
    double avgLight = 0;
    double ratio = 0;
    double[] lineTheta = new double[4];
    double[] position = new double[3];
    double thetaX, thetaY, x, y, theta, dTheta, distance;
    int linesCrossed = 0;
    long lastLine = 0;
    long curLine = 0;
    
    // Rotate 45° to find the standard reflectance of the tile
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    rightMotor.rotate(-convertAngle(RADIUS, TRACK, 45.0), true);
    leftMotor.rotate(convertAngle(RADIUS, TRACK, 45.0), false);
    
    standardization();
    
    // Rotate 360° and record position of 4 lines crossed 
    rightMotor.rotate(-convertAngle(RADIUS, TRACK, 359.9), true);
    leftMotor.rotate(convertAngle(RADIUS, TRACK, 359.9), true);
    
    while (leftMotor.isMoving() || rightMotor.isMoving()) {
      theta = detectLines(STANDARD, 0);
      if (theta != -1 && linesCrossed < 4) {
    	if (linesCrossed > 0) {
          curLine = System.currentTimeMillis();
          if (lastLine - curLine > LINE_FILTER) {
        	lineTheta[linesCrossed] = theta;
        	Sound.beep();
            linesCrossed++;
            lastLine = curLine;
          }
        } else if (linesCrossed == 0) {
          lastLine = System.currentTimeMillis();
          Sound.beep();
          Sound.beep();
          lineTheta[linesCrossed] = theta;
          linesCrossed++;
        }
    	Sound.beep();
        Sound.beep();
        lineTheta[linesCrossed] = theta;
        linesCrossed++;
      } else if (theta != -1) {
    	linesCrossed++;
      }
    }
    
    // Calculate position relative to the origin based on position of 4 lines crossed
    // We assume it crosses the lines in this order: -x, +y, +x, -y
    thetaX = ((lineTheta[0] - lineTheta[2]) + 180) % 180;
    thetaY = ((lineTheta[1] - lineTheta[3]) + 180) % 180;
    x = LSRADIUS * Math.cos(Math.toRadians(thetaY / 2));
    y = LSRADIUS * Math.cos(Math.toRadians(thetaX / 2));
    
    if (lineTheta[0] < 270.0 && lineTheta[2] > 90.0) { // 1st or 2nd quadrant
      if (lineTheta[1] < 359.9 && lineTheta[3] > 180.0) { // 1st quadrant
        // nothing happens, x and y are correct sign
      } else { // 2nd quadrant
        x = -x;
      }
    } else { // 3rd or 4th quadrant
      if (lineTheta[1] < 359.9 && lineTheta[3] > 180.0) { // 4th quadrant
        y = -y;
      } else { // 3rd quadrant
        x = -x;
        y = -y;
      }
    }
    Odometer.setX(lineTheta[0]);
    Odometer.setY(lineTheta[2]);
    
    // Turn towards the origin
    position = Odometer.getPosition();
    dTheta = position[2] - calculateTheta(-x, -y);
    if (dTheta < -180) {
      dTheta += 360;
    } else if (dTheta > 180) {
      dTheta -= 360;
    }
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    rightMotor.rotate(-convertAngle(RADIUS, TRACK, dTheta), true);
    leftMotor.rotate(convertAngle(RADIUS, TRACK, dTheta), false);
    
    // Drive straight to the origin
    distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
	leftMotor.setSpeed(LEFT_SPEED);
    rightMotor.setSpeed(RIGHT_SPEED);
	leftMotor.rotate(convertDistance(RADIUS, distance), true);
    rightMotor.rotate(convertDistance(RADIUS, distance), false);
    
    // Turn to the 0°
    turnTo(0);
    
    // Check angle accuracy
    for(int i = 0; i < SAMPLE_NUM; i++) {
      light.fetchSample(lightData, 0);
      sumLight += lightData[0]*100;
    }
    avgLight = (sumLight+0.0)/(SAMPLE_NUM+0.0);
    ratio = (avgLight/STANDARD);
    if (ratio >= 1.1 || ratio <= 0.9) { // We have successfully placed the robot on the origin
      return; 
    } else {
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
      rightMotor.rotate(-convertAngle(RADIUS, TRACK, 45), true);
      leftMotor.rotate(convertAngle(RADIUS, TRACK, 45), false);
      leftMotor.rotate(-convertAngle(RADIUS, TRACK, 90), true);
      rightMotor.rotate(convertAngle(RADIUS, TRACK, 90), true);
      while(leftMotor.isMoving() || rightMotor.isMoving()) {
        detectLines(STANDARD, 1); // Stops when it detects a line
      }
      Odometer.setT(0);
      return;
    }
    
  }
  
  /**
   * This method allows the conversion of a distance to the total rotation of each wheel needed to
   * cover that distance.
   * 
   * @param radius
   * @param distance
   * @return
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }
  
  /**
   * This method allows the conversion of a angle to the total rotation of each wheel needed to
   * cover that distance.
   * 
   * @param radius
   * @param width
   * @param angle
   * @return
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  
  /**
   * This method calculates the smallest angle to turn to
   * 
   * @param dX The distance we want to go in the x direction
   * @param dY The distance we want to go in the y direction
   * @return
   */
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
   * This method polls the light sensor and acts according to the mode it is in when it sees a line.
   * 
   * @param standard The standard reflectance of a tile
   * @param mode If mode = 1, records the current angular position of the robot. If mode = 0, stops the motors.
   * @return If line is encountered, returns its current angular position, otherwise returns -1
   */
  private double detectLines(double standard, int mode) {
    double result = -1;
    long start, end;
    double sumLight = 0;
    double avgLight = 0;
    double ratio = 0;
    double[] position = new double[3];
    
    start = System.currentTimeMillis();
    
	for(int i = 0; i < SAMPLE_NUM; i++) {
	  light.fetchSample(lightData, 0);
	  sumLight += lightData[0]*100;
	}
    avgLight = (sumLight+0.0)/(SAMPLE_NUM+0.0);
    ratio = (avgLight/standard);
	
    if (ratio >= 1.15 || ratio <= 0.85) {
      Sound.beep();
 	  if (mode == 0) {
        position = Odometer.getPosition();
        result = position[2];
 	  } else if (mode == 1) {
 		Sound.beep();
 	    rightMotor.stop();
 	    leftMotor.stop();
 	  }
	}
    avgLight = 0;
    ratio = 0;
	      
    end = System.currentTimeMillis();
	if (end - start < SAMPLE_PERIOD) {
	  try {
	    Thread.sleep(SAMPLE_PERIOD - (end - start));
	  } catch (InterruptedException e) {
	    // there is nothing to be done here
	  }
	}
    return result;
  }
}
