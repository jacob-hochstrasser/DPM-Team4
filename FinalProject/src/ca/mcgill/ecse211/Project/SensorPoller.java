package ca.mcgill.ecse211.Project;

/**
 * This is the interface for sensors.
 * @author Pengnan Fan
 */

public interface SensorPoller {
	
	/**
	 * This method returns the average measurement of a sensor.
	 * 
	 * @param MEASURING_SCOPE
	 * This is a int value for the number of measurements for averaging.
	 * 
	 * @return
	 * This method returns the average result.
	 */
	public float getData(int MEASURING_SCOPE);
	
	/**
	 * This method immediately returns a measurement.
	 * @return
	 * This method returns the result for one measurement.
	 */
	public float getData();
}
