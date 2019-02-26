package ca.mcgill.ecse211.lab5;

import java.text.DecimalFormat;

import ca.mcgill.ecse211.lab5.Odometer;
import lejos.hardware.lcd.TextLCD;

public class Display extends Thread{
	private static TextLCD SCREEN;
	private double[] position;
	private static final int DISPLAY_PERIOD = 50;
	
	public Display(TextLCD SCREEN) {
		this.SCREEN = SCREEN;
	}
	
	public void run() {   
		long updateStart, updateEnd;
		// Reset screen 
		SCREEN.clear();
		
		do {
	      updateStart = System.currentTimeMillis();
	      
	      // Retrieve x, y and theta information
	      position = Odometer.getPosition();
	      
	      // Print x,y, and theta information
	      DecimalFormat numberFormat = new DecimalFormat("######0.00");
	      SCREEN.drawString("<X> " + numberFormat.format(position[0]), 0, 1);
	      SCREEN.drawString("<Y> " + numberFormat.format(position[1]), 0, 2);
	      SCREEN.drawString("<Theta> " + numberFormat.format(position[2]), 0, 3);
			      
	      // this ensures that the data is updated only once every period
	      updateEnd = System.currentTimeMillis();
	      if (updateEnd - updateStart < DISPLAY_PERIOD) {
	        try {
			  Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
			} catch (InterruptedException e) {}
		  }
		} while (true);

	  }
}
