package ca.mcgill.ecse211.testing;

public class DrivingTest {
	/**
	 * This class is used for driving tests.
	 * 
	 * @author Pengnan Fan
	 */
	
	//-----<Important Constant>-----//
	private static final double TILE_SIZE = 30.48;//cm
	
	public static void main(String[] args) {
		MotorController mc = MotorController.getMotorController("B", "D");
		
		//Square Drive
		squareDriver(mc);
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method drives the robot to move as a 3*3 square.
	 * 
	 * @param mc:MotorController
	 * This input is the MotorController.
	 */
	public static void squareDriver(MotorController mc) {
		for(int i = 0; i<4; i++) {
			mc.moveForward(TILE_SIZE*3);
			mc.turnLeft(90);
		}
	}
	
}
