package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Project {
	//-----<Important Constant>-----//
	private static final int LOCALIZING_INTERVAL = 25;//ms
	
    private static final EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
    private static final EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    
    //-----<Sensor>-----//
    //Light Sensors//
    private static final SensorPoller LOCALIZING_LEFT = new LightSensorPoller("S1", LOCALIZING_INTERVAL, false);
    private static final SensorPoller LOCALIZING_RIGHT = new LightSensorPoller("S4", LOCALIZING_INTERVAL, false);
    
    
    //-----<Navigation>-----//
    private static Navigation NAVI;
    
    private static final TextLCD LCD = LocalEV3.get().getTextLCD();


    public static void main(String[] args) {
    	/*
        int buttonChoice;
        
        SampleProvider locLSRed = LOCALIZING_LIGHT_SENSOR.getMode("Red");
        SampleProvider idLSRGB = LOCALIZING_LIGHT_SENSOR.getMode("RGB");
        int startingCorner = 0, targetColor = 1;
        int[] ll = new int[2];
        int[] ur = new int[2];

        do {
            LCD.clear();
            LCD.drawString("Starting corner: ", (LCD.getTextWidth()/2) - 9, 0);
            LCD.drawString("< " + startingCorner + " >" , LCD.getTextWidth() - 3, 2);
            buttonChoice = Button.waitForAnyPress();  
            if(buttonChoice == Button.ID_LEFT && startingCorner > 0) {
                startingCorner--;
            } else if(buttonChoice == Button.ID_RIGHT && startingCorner < 3){
                startingCorner++;
            }
        } while(buttonChoice != Button.ID_ENTER);
        
        do {
            LCD.clear();
            LCD.drawString("Lower Left X: ", (LCD.getTextWidth()/2) - 8, 0);
            LCD.drawString("< " + ll[0] + " >", (LCD.getTextWidth()/2) - 3, 2);
            buttonChoice = Button.waitForAnyPress();
            if(buttonChoice == Button.ID_LEFT && ll[0] > 0) {
                ll[0]--;
            } else if(buttonChoice == Button.ID_RIGHT && ll[0] < 8) {
                ll[0]++;
            }
        } while(buttonChoice != Button.ID_ENTER);
        
        do {
            LCD.clear();
            LCD.drawString("Lower Left Y: ", (LCD.getTextWidth()/2) - 8, 0);
            LCD.drawString("< " + ll[1] + " >", (LCD.getTextWidth()/2) - 3, 2);
            buttonChoice = Button.waitForAnyPress();
            if(buttonChoice == Button.ID_LEFT && ll[1] > 0) {
                ll[1]--;
            } else if(buttonChoice == Button.ID_RIGHT && ll[1] < 8) {
                ll[1]++;
            }
        } while(buttonChoice != Button.ID_ENTER);

        do {
            LCD.clear();
            LCD.drawString("Upper Right X: ", (LCD.getTextWidth()/2) - 8, 0);
            LCD.drawString("< " + ur[0] + " >", (LCD.getTextWidth()/2) - 3, 2);
            buttonChoice = Button.waitForAnyPress();
            if(buttonChoice == Button.ID_LEFT && ur[0] > 0) {
                ur[0]--;
            } else if(buttonChoice == Button.ID_RIGHT && ur[0] < 8) {
                ur[0]++;
            }
        } while(buttonChoice != Button.ID_ENTER);
        
        do {
            LCD.clear();
            LCD.drawString("Upper Right Y: ", (LCD.getTextWidth()/2) - 8, 0);
            LCD.drawString("< " + ur[1] + " >", (LCD.getTextWidth()/2) - 3, 2);
            buttonChoice = Button.waitForAnyPress();
            if(buttonChoice == Button.ID_LEFT && ur[1] > 0) {
                ur[1]--;
            } else if(buttonChoice == Button.ID_RIGHT && ur[1] < 8) {
                ur[1]++;
            }
        } while(buttonChoice != Button.ID_ENTER);
        
        do {
            LCD.clear();
            LCD.drawString("Target Color: ", (LCD.getTextWidth()/2) - 7, 0);
            //LCD.drawString("< " + targetColor + ": " + Identifier.TARGET_COLOR.tcToString(targetColor) + " >", (LCD.getTextWidth()/2) - (3 + Identifier.TARGET_COLOR.tcToString(targetColor).length()/2), 2);
            buttonChoice = Button.waitForAnyPress();
            if(buttonChoice == Button.ID_LEFT && targetColor > 1) {
                targetColor--;
            } else if(buttonChoice == Button.ID_RIGHT && targetColor <4) {
                targetColor++;
            }
        } while(buttonChoice != Button.ID_ENTER);
        */
        NAVI = new Navigation(LEFT_MOTOR, RIGHT_MOTOR, LOCALIZING_LEFT, LOCALIZING_RIGHT, 1);
        NAVI.demo();
        
    }

}
