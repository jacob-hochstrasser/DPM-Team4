package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * This class implements the sensor controller for ECSE211 Lab 3 on the 
 * Lego EV3 platform.
 * 
 *  @author Pengnan Fan
 *  @debug Pengnan Fan
 *  
 */

public class SensorController extends Thread {
	
  //------<Important Constant>------//
  private static final long PERIOD = 100;//ms
  private static final int US_FILTER_SCOPE = 20;
  private static final int LIGHT_FILTER_SCOPE = 5;
  private static final int SENSITIVITY = 25;//for light sensor
  public static int counter = 0;
  private static int NAVI_MODE;
  private static int INFTY;
  private static double[][] Xs;
  private static double[][] Ys;
  
  //------<Sensors>------//
  private static SampleProvider uData = 
    (new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"))).getMode("Distance");
  private static SampleProvider lData = 
    (new EV3ColorSensor(LocalEV3.get().getPort("S1"))).getRedMode();
  private static float[] distanceData;
  private static float[] lightData;
  
  //------<Navigation>------//
  private static Navigation NAVI;
  
  private static final TextLCD SCREEN = LocalEV3.get().getTextLCD();
  
  /**
   * This is the constructor of this class
   * 
   * @param NAVI. This is the Navigation used to drive the robot
   * @param NAVI_MODE. This is the index of map (0 - 3)
   * @param Xs. This is an 2d-array of x coordinates
   * @param Ys. This is an 2d-array of y coordinates
   * @param INFTY. This is the maximum distance between the robot and the wall in centimeter
   */
  public SensorController(Navigation NAVI, int NAVI_MODE, double[][] Xs, double[][] Ys, int INFTY) {
	//Initialize navigation mode
	this.NAVI = NAVI;
	this.INFTY = INFTY;
	
	//Initialize NAVI_MODE
	this.NAVI_MODE = NAVI_MODE;
	
	//Initialize coordinates
	this.Xs = Xs;
	this.Ys = Ys;
	
	//Initialize data array
	distanceData = new float[uData.sampleSize()];
	lightData = new float[lData.sampleSize()];
  }
  
  /**
   * This is required by Thread.start(). It runs the sensor controller
   */
  public void run() {
	Sound.beep();
	boolean obstacle = false;
	int distance = 0;
	while(counter<5) {
	  obstacle = lightWithFilter();
	  distance = distanceWithFilter();
	  NAVI.advancedTravelTo(Xs[NAVI_MODE][counter], Ys[NAVI_MODE][counter], obstacle, distance);
	}
	NAVI.turnTo(0);
  }
  
  private boolean lightWithFilter() {
	float sum = 0;
	for(int i = 0; i<LIGHT_FILTER_SCOPE;i++) {
	  lData.fetchSample(lightData, 0);
	  sum+=lightData[0]*1000;
	}
	return (sum/5)>=SENSITIVITY;
  }
  
  private int distanceWithFilter() {
	for(int i = 0; i<US_FILTER_SCOPE;) {
	  uData.fetchSample(distanceData, 0);
	  if(distanceData[0]*100<INFTY) {return (int)Math.round(distanceData[0]*100);}
	  i++;
	}
	return INFTY;
  }
  
  
  
}
