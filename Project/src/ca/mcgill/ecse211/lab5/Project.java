package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Project {
    /*<--------------Static Root Class Variables -- Motors, Sensors, and Odometers that will be used by multiple classes -------------------> */
    /*<--------------Motor Port Mappings: LMotor -> B, Scanner -> C, RMotor -> D -----------------------------------------------------------> */
    /*<--------------Sensor Port Mappings: LLocalizingLS -> S1, USSensor -> S2, IDLightSensor -> S3 RLocalizingLS -> S4 --------------------> */
    //-----<Important Constants>-----//
    /**
     * Polling interval for light sensor
     */
    private static final int LOCALIZING_INTERVAL_LIGHT = 25;//ms
    /**
     * Polling interval for US Sensor
     */
    private static final int LOCALIZING_INTERVAL_ULTRASONIC = 25;//ms
    /**
     * Radius of the wheels
     */
    private static final double RADIUS = 2.1;
    /**
     * Width of wheel base
     */
    private static final double TRACK = 9.2;
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
     * Static variable for left localizing light sensor poller
     */
    private static final SensorPoller LOCALIZING_LEFT = new LightSensorPoller("S1", LOCALIZING_INTERVAL_LIGHT, false);
    /**
     * Static variable for right localizing light sensor poller
     */
    private static final SensorPoller LOCALIZING_RIGHT = new LightSensorPoller("S4", LOCALIZING_INTERVAL_LIGHT, false);
    /**
     * Static variable for ultrasonic sensor poller
     */
    private static final SensorPoller ULTRASONIC = new UltrasonicSensorPoller("S2", LOCALIZING_INTERVAL_ULTRASONIC);
    /**
     * Static variable for LCD display of brick
     */
    private static final TextLCD LCD = LocalEV3.get().getTextLCD();
    /**
     * Odometer to keep track of where the robot is
     */
    private final static Odometer odo = new Odometer(LEFT_MOTOR, RIGHT_MOTOR, RADIUS, TRACK);
    /**
     * Navigation object to handle the logic for where the robot should go
     */
    private static final int IDENTIFYING_INTERVAL = 0;//ms

    //-----<Sensor>-----//
    private static final SampleProvider IDENTIFYING = (new EV3ColorSensor(LocalEV3.get().getPort("S3"))).getRGBMode();

    //-----<Navigation>-----//
    private static final Navigation NAVI;

    //-----<Display>----//
    private static final Display DIS = new Display(LCD);

    //-----<Identifier>-----//
    private static final Identifier IDFER;


    public static void main(String[] args) {

        int buttonChoice;

        /*--- Variables to store the parameters specified in the Lab 5 instructions ---*/
        /*--- Starting corner -> [0,3] Target color -> {1 - Blue, 2 - Green, 3 - Yellow, 4 - Red} Lower left of search corner -> ([0,8], [0,8]) Upper right of search corner -> ([0,8], [0,8]) ---*/
        int mode = 0, startingCorner = 0, targetColor = 1;
        int[] ll = new int[2];
        int[] ur = new int[2];

        /*--- Pick mode (0 - Data collection, 1 - Color id, 2 - Field test, 3 - Debug) ---*/
        do {
            LCD.clear();
            LCD.drawString("Mode: ", (LCD.getTextWidth()/2)-3, 0);
            LCD.drawString("< " + mode + " >", (LCD.getTextWidth()/2)-3, 2);
            buttonChoice = Button.waitForAnyPress();
            if(buttonChoice == Button.ID_LEFT && mode > 0) {
                mode--;
            } else if(buttonChoice == Button.ID_RIGHT && mode < 3) {
                mode++;
            } else if(buttonChoice == Button.ID_ESCAPE) {
                System.exit(0);
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

        LCD.clear();
        Identifier identifier = new Identifier(SCANNER, targetColor, idLSRGB, 1000, LCD);
        if(mode == 0) {
            LCD.drawString("Press enter", (LCD.getTextWidth()/2) - 5, 0);
            LCD.drawString("to scan", (LCD.getTextWidth()/2) - 3, 1);
            do {
                buttonChoice = Button.waitForAnyPress(1);
                if(buttonChoice == Button.ID_ENTER) {
                    LCD.clear();
                    identifier.computeReferences();
                }
            } while(buttonChoice != Button.ID_ESCAPE);
            System.exit(0);
        } else if(mode == 1) {
            LCD.clear();
            do {
                identifier.idColor();
                buttonChoice = Button.waitForAnyPress(1);
            } while(buttonChoice != Button.ID_ESCAPE);
            System.exit(0);
        } else if (mode == 2) {

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

            odo.start();
            DIS.start();
            int buttonChoice = 0;
            NAVI = new Navigation(0, new int[] {3,3}, new int[] {6,6});
            System.out.println("Press enter to continue");
            do {
                buttonChoice = Button.waitForAnyPress();
            } while(buttonChoice != Button.ID_ENTER);
            Sound.beep();
            LCD.clear();
            NAVI.demo();   
        }
    }
