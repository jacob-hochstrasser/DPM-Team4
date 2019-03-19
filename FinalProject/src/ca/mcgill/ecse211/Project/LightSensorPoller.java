package ca.mcgill.ecse211.Project;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class is the poller for light sensors.
 * @author Pengnan Fan
 */

public class LightSensorPoller implements SensorPoller{

	//-----<LightSensor>-----//
	private Port lsPort;
	private SensorModes lightSensor;
	private SampleProvider light;
	
	//-----<Important Constant>-----//
	private int TIME_INTERVAL;
	
	//-----<Data>-----//
	private float[] lsData = new float[1];
	
	/**
	 * This is the constructor of a light sensor poller.
	 * 
	 * @param port
	 * This input suggests the string of the port of the sensor.
	 *  
	 * @param TIME_INTERVAL
	 * This input suggests the int value for time interval between two measurements.
	 * 
	 * @param isColorIdentification
	 * This input suggests the boolean value for if the sensor is set to be color detection mode.
	 */
	public LightSensorPoller(String port, int TIME_INTERVAL, boolean isColorIdentification) {
		Sound.setVolume(100);
		this.TIME_INTERVAL = TIME_INTERVAL;
		lsPort = LocalEV3.get().getPort(port);
		lightSensor = new EV3ColorSensor(lsPort);
		if (isColorIdentification) {
			light = ((EV3ColorSensor)lightSensor).getRGBMode();
		} else {
			light = ((EV3ColorSensor)lightSensor).getRedMode();
		}
		try {Thread.sleep(500);} catch (InterruptedException e) {}
	}
	
	/**
	 * This method returns the average measurement of the light sensor.
	 * 
	 * @param MEASURING_SCOPE
	 * This is a int value for the number of measurements for averaging.
	 * 
	 * @return 
	 * This method returns the average result.
	 */
	@Override
	public float getData(int MEASURING_SCOPE) {
		if(MEASURING_SCOPE<=1) {
			return getData();
		}
		float measure = 0;
		for(int i = 0; i<MEASURING_SCOPE; i++) {
			long Start = System.currentTimeMillis();
			light.fetchSample(lsData, 0);
			measure += lsData[0]*100;
			long End = System.currentTimeMillis();
			if (End - Start < TIME_INTERVAL) {
				try {Thread.sleep(TIME_INTERVAL - (End - Start));} catch (InterruptedException e) {}
			}
		}
		return measure/MEASURING_SCOPE;
	}
	
	/**
	 * This method returns the result for one measurement.
	 */
	@Override
	public float getData() {
		light.fetchSample(lsData, 0);
		return lsData[0]*100;
	}
}
