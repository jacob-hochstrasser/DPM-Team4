package ca.mcgill.ecse211.Project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This is the poller class for ultrasonic sensor.
 * @author Pengnan Fan
 */

public class UltrasonicSensorPoller implements SensorPoller{
	//-----<LightSensor>-----//
    /**
     * The port into which the referenced ultrasonic sensor is plugged
     */
	private Port usPort;
	/**
	 * The ultrasonic sensor mode to use
	 */
	private SensorModes ultrasonicSensor;
	/**
	 * The sample provider for the referenced ultrasonic sensor
	 */
	private SampleProvider ultrasonic;
	
	//-----<Important Constant>-----//
	/**
	 * The time interval between samples (in ms)
	 */
	private int TIME_INTERVAL;
	/**
	 * The value that corresponds to an infinity (or null) reading
	 */
	private int INFTY = 255;
	/**
	 * The minimum number of infinity distance samples that must be received to be considered valid.
	 */
	private int FILTER_SCOPE = 20;
	
	//-----<Data>-----//
	/**
	 * A buffer to store the sample data in
	 */
	private float[] usData = new float[1];
	
	/**
	 * This is the constructor of an ultrasonic sensor poller
	 * 
	 * @param port
	 * This input suggests the string of the port of the sensor.
	 *  
	 * @param TIME_INTERVAL
	 * This input suggests the int value for time interval between two measurements.
	 * 
	 */
	public UltrasonicSensorPoller(String port, int TIME_INTERVAL) {
		Sound.setVolume(100);
		this.TIME_INTERVAL = TIME_INTERVAL;
		usPort = LocalEV3.get().getPort(port);
		while(ultrasonicSensor == null) {
		    try {
		        ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		    } catch(IllegalArgumentException e) {
		        Sound.beep();
		        System.out.println(e.getMessage());
		        Button.waitForAnyPress();
		    }
		}
		
		ultrasonic = ultrasonicSensor.getMode("Distance");
	}
	
	/**
	 * This method returns the average measurement of the ultrasonic sensor.
	 * 
	 * @param MEASURING_SCOPE
	 * This is a int value for the number of measurements for averaging.
	 * 
	 * @return 
	 * This method returns the average result.
	 */
	@Override
	public float getData(int MEASURING_SCOPE) {
		float measure = 0;
		for(int i = 0; i<MEASURING_SCOPE; i++) {
			long Start = System.currentTimeMillis();
			ultrasonic.fetchSample(usData, 0);
			measure += usData[0]*100;
			long End = System.currentTimeMillis();
			if (End - Start < TIME_INTERVAL) {
				try {Thread.sleep(TIME_INTERVAL - (End - Start));} catch (InterruptedException e) {}
			}
		}
		return measure/MEASURING_SCOPE;
	}
	/**
	 * This method immediately returns one measurement (with filter).
	 */
	@Override
	public float getData() {
		for (int i = 0; i<FILTER_SCOPE; i++) {
			ultrasonic.fetchSample(usData, 0);
			float data = usData[0]*100;
			if (data<INFTY) {
				//System.out.println(data);
				return data;
			}
		}
		//System.out.println(INFTY);
		return INFTY;
	}
}
