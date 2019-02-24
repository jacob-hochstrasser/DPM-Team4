package ca.mcgill.ecse211.lab5;

public interface SensorPoller {
	
	/**
	 * This method returns the average measurement of a sensor.
	 * 
	 * @param MEASURING_SCOPE:int = number of measurements
	 * @return average measurement
	 */
	public float getData(int MEASURING_SCOPE);
	
	/**
	 * This method immediately returns a measurement
	 */
	public float getData();
}
