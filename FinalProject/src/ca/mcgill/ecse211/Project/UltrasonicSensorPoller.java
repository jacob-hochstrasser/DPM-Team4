package ca.mcgill.ecse211.Project;

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
	private Port usPort;
	private SensorModes ultrasonicSensor;
	private SampleProvider ultrasonic;
	
	//-----<Important Constant>-----//
	private int TIME_INTERVAL;
	private int INFTY = 255;
	private int FILTER_SCOPE = 20;
	
	//-----<Data>-----//
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
		ultrasonicSensor = new EV3UltrasonicSensor(usPort);
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
