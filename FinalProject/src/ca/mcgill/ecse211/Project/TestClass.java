package ca.mcgill.ecse211.Project;

public class TestClass {
	public static void main(String[] args) {
		Navigation nv = Navigation.getNavigation("B", "D");
		Odometer odo = new Odometer();
		odo.start();
		//nv.demo();
		nv.turnRight(90);
	}
}
