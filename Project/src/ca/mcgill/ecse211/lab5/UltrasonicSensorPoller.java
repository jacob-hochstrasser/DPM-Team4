package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class UltrasonicSensorPoller implements SensorPoller{
	//-----<LightSensor>-----//
	private Port usPort;
	private SensorModes ultrasonicSensor;
	private SampleProvider ultrasonic;
	
	//-----<Important Constant>-----//
	private int TIME_INTERVAL;
	
	//-----<Data>-----//
	private float[] usData = new float[1];
	
	/**
	 * This is the constructor of an ultrasonic sensor poller
	 * 
	 * @param port:String = port of the sensor
	 * @param TIME_INTERVAL:int = Time interval between each measurement
	 */
	public UltrasonicSensorPoller(String port, int TIME_INTERVAL) {
		Sound.setVolume(100);
		this.TIME_INTERVAL = TIME_INTERVAL;
		usPort = LocalEV3.get().getPort(port);
		ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		ultrasonic = ultrasonicSensor.getMode("Distance");
		Sound.beep();
	}
	
	/**
	 * This method returns the average measurement of the ultrasonic sensor.
	 * 
	 * @param MEASURING_SCOPE:int = number of measurements
	 * @return average measurement
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
	 * This method immediately returns a measurement
	 */
	@Override
	public float getData() {
		ultrasonic.fetchSample(usData, 0);
		return usData[0]*100;
	}
}
