package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Project {
    /*<--------------Static Root Class Variables -- Motors, Sensors, and Odometers that will be used by multiple classes -------------------> */
    /*<--------------Motor Port Mappings: LMotor -> B, Scanner -> C, RMotor -> D -----------------------------------------------------------> */
    /*<--------------Sensor Port Mappings: LLocalizingLS -> S1, USSensor -> S2, IDLightSensor -> S3 RLocalizingLS -> S4 --------------------> */
    /**
     * Static variable for left motor
     */
    private static final EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
    /**
     * Static variable for right motor
     */
    private static final EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    /**
     * Static variable for scanning motor
     */
    private static final EV3MediumRegulatedMotor SCANNER = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
    /**
     * Static variable for LCD display of brick
     */
    private static final TextLCD LCD = LocalEV3.get().getTextLCD();

    /**
     * Main method of Lab 5. Provides a menu interface for choosing field test parameters and initializes/prepares all hardware needed.
     * @param args
     */
    public static void main(String[] args) {
        
        int buttonChoice; // integer to track which button was pressed
        
        /*--- Setting up the sensors ---*/
        @SuppressWarnings("resource")
        SensorModes LOCALIZING_LIGHT_SENSOR = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
        @SuppressWarnings("resource")
        SensorModes ULTRASONIC_SENSOR = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
        @SuppressWarnings("resource")
        SensorModes IDENTIFYING_LIGHT_SENSOR = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
        
        SampleProvider locLSRed = LOCALIZING_LIGHT_SENSOR.getMode("Red");
        SampleProvider idLSRGB = LOCALIZING_LIGHT_SENSOR.getMode("RGB");
        SampleProvider usDistance = ULTRASONIC_SENSOR.getMode("Distance");
        
        float[] lLSData = new float[locLSRed.sampleSize()];
        float[] usData = new float[usDistance.sampleSize()];
        
        /*--- Variables to store the parameters specified in the Lab 5 instructions ---*/
        /*--- Starting corner -> [0,3] Target color -> {1 - Blue, 2 - Green, 3 - Yellow, 4 - Red} Lower left of search corner -> ([0,8], [0,8]) Upper right of search corner -> ([0,8], [0,8]) ---*/
        int startingCorner = 0, targetColor = 1;
        int[] ll = new int[2];
        int[] ur = new int[2];

        /*--- Pick the starting corner ---*/
        do {
            LCD.clear();
            LCD.drawString("Starting corner: ", (LCD.getTextWidth()/2) - 8, 0);
            LCD.drawString("< " + startingCorner + " >" , (LCD.getTextWidth()/2) - 3, 2);
            buttonChoice = Button.waitForAnyPress();  
            if(buttonChoice == Button.ID_LEFT && startingCorner > 0) {
                startingCorner--;
            } else if(buttonChoice == Button.ID_RIGHT && startingCorner < 3){
                startingCorner++;
            } else if(buttonChoice == Button.ID_ESCAPE) {
                System.exit(0);
            }
        } while(buttonChoice != Button.ID_ENTER);

        /*--- Pick lower left x ---*/
        do {
            LCD.clear();
            LCD.drawString("Lower Left X: ", (LCD.getTextWidth()/2) - 8, 0);
            LCD.drawString("< " + ll[0] + " >", (LCD.getTextWidth()/2) - 3, 2);
            buttonChoice = Button.waitForAnyPress();
            if(buttonChoice == Button.ID_LEFT && ll[0] > 0) {
                ll[0]--;
            } else if(buttonChoice == Button.ID_RIGHT && ll[0] < 8) {
                ll[0]++;
            } else if(buttonChoice == Button.ID_ESCAPE) {
                System.exit(0);
            }
        } while(buttonChoice != Button.ID_ENTER);

        /*--- Pick lower left y ---*/
        do {
            LCD.clear();
            LCD.drawString("Lower Left Y: ", (LCD.getTextWidth()/2) - 8, 0);
            LCD.drawString("< " + ll[1] + " >", (LCD.getTextWidth()/2) - 3, 2);
            buttonChoice = Button.waitForAnyPress();
            if(buttonChoice == Button.ID_LEFT && ll[1] > 0) {
                ll[1]--;
            } else if(buttonChoice == Button.ID_RIGHT && ll[1] < 8) {
                ll[1]++;
            } else if(buttonChoice == Button.ID_ESCAPE) {
                System.exit(0);
            }
        } while(buttonChoice != Button.ID_ENTER);

        /*--- Pick upper right x ---*/
        do {
            LCD.clear();
            LCD.drawString("Upper Right X: ", (LCD.getTextWidth()/2) - 8, 0);
            LCD.drawString("< " + ur[0] + " >", (LCD.getTextWidth()/2) - 3, 2);
            buttonChoice = Button.waitForAnyPress();
            if(buttonChoice == Button.ID_LEFT && ur[0] > 0) {
                ur[0]--;
            } else if(buttonChoice == Button.ID_RIGHT && ur[0] < 8) {
                ur[0]++;
            } else if(buttonChoice == Button.ID_ESCAPE) {
                System.exit(0);
            }
        } while(buttonChoice != Button.ID_ENTER);

        /*--- Pick upper right y ---*/
        do {
            LCD.clear();
            LCD.drawString("Upper Right Y: ", (LCD.getTextWidth()/2) - 8, 0);
            LCD.drawString("< " + ur[1] + " >", (LCD.getTextWidth()/2) - 3, 2);
            buttonChoice = Button.waitForAnyPress();
            if(buttonChoice == Button.ID_LEFT && ur[1] > 0) {
                ur[1]--;
            } else if(buttonChoice == Button.ID_RIGHT && ur[1] < 8) {
                ur[1]++;
            } else if(buttonChoice == Button.ID_ESCAPE) {
                System.exit(0);
            }
        } while(buttonChoice != Button.ID_ENTER);

        /*--- Pick target color ---*/
        do {
            LCD.clear();
            LCD.drawString("Target Color: ", (LCD.getTextWidth()/2) - 7, 0);
            LCD.drawString("< " + targetColor + ": " + Identifier.TARGET_COLOR.tcToString(targetColor) + " >", (LCD.getTextWidth()/2) - (3 + Identifier.TARGET_COLOR.tcToString(targetColor).length()/2), 2);
            buttonChoice = Button.waitForAnyPress();
            if(buttonChoice == Button.ID_LEFT && targetColor > 1) {
                targetColor--;
            } else if(buttonChoice == Button.ID_RIGHT && targetColor <4) {
                targetColor++;
            } else if(buttonChoice == Button.ID_ESCAPE) {
                System.exit(0);
            }
        } while(buttonChoice != Button.ID_ENTER);
        
        Identifier identifier = new Identifier(SCANNER, targetColor, idLSRGB, 100);

        while (Button.waitForAnyPress() != Button.ID_ESCAPE);
        System.exit(0);
    }

    
}
