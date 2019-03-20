package ca.mcgill.ecse211.Project;

import java.text.DecimalFormat;
import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * Main class being run on the robot. It establishes a wifi connection and receives data from it, controls the logic of the main program, and initializes all the classes and variables.
 *  
 * @author Jake
 *
 */
public class BetaDemoApplication {

    /**
     * The IP address of the server (ie laptop) communicating with the robot.
     */
    private static final String SERVER_IP = "192.168.2.35";

    /**
     * The number of the team responsible for the robot.
     */
    private static final int TEAM_NUMBER = 1;

    /**
     * Enables printing of messages during wifi connection for debugging
     */
    private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;

    /**
     * The motor used to scan around cans.
     */
    private static final EV3MediumRegulatedMotor SCANNER = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

    /**
     * The Sample Provider for the identifying color sensor
     */
    private static final SampleProvider IDRGB = new EV3ColorSensor(LocalEV3.get().getPort("S4")).getRGBMode();

    /**
     * The text LCD of the containing EV3 brick
     */
    public static final TextLCD LCD = LocalEV3.get().getTextLCD();

    /**
     * String identifying the port of the left motor
     */
    private static final String LEFT_MOTOR = "B";

    /**
     * String identifying the port of the right motor
     */
    private static final String RIGHT_MOTOR = "D";

    /**
     * The navigation class that handles all methods for localizing, moving, and orienting the robot
     */
    private static final Navigation NAV = Navigation.getNavigation(LEFT_MOTOR, RIGHT_MOTOR);

    /**
     * The odometer that keeps track of the robot's position and heading
     */
    private static final Odometer ODO = new Odometer();

    /**
     * The identifier class that handles can color identification
     */
    public static Identifier ID;

    /**
     * Enables printing of statements during program execution for debugging
     */
    protected static final boolean DEBUG = false;

    /**
     * Formats decimal values for printing debugging statements
     */
    protected static final DecimalFormat DF = new DecimalFormat();

    /**
     * Main execution thread of the program.
     */
    @SuppressWarnings("rawtypes")
    public static void main(String[] args) {

        // limit number of decimals for formatted numbers to two decimal places
        DF.setMaximumFractionDigits(2);
        
        // Initialize WifiConnection class
        WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

        /*--- Variables to store data necessary for the demo ---*/
        int[] ll = new int[2]; // lower left coordinates of the team's home zone
        int[] ur = new int[2]; // upper right coordinates of the team's home zone
        int[] island_ll = new int[2]; // lower left coordinates of the island
        int[] island_ur = new int[2]; // upper right coordinates of the island
        int[] tunnel_ll = new int[2]; // lower left coordinates of the tunnel
        int[] tunnel_ur = new int[2]; // upper right coordinates of the tunnel
        int[] sz_ll = new int[2]; // lower left coordinates of the team's search zone
        int[] sz_ur = new int[2]; // upper right coordinates of the team's search zone
        int startCorner = 0, targetColor = 0; // starting corner of the competition table, target can color (1 = blue, 2 = green, 3 = yellow, 4 = red)

        /*--- Collect data from the server ---*/
        try {
            Map data = conn.getData();
            ll[0] = ((Long) data.get("Red_LL_x")).intValue();
            ll[1] = ((Long) data.get("Red_LL_y")).intValue();
            ur[0] = ((Long) data.get("Red_UR_x")).intValue();
            ur[1] = ((Long) data.get("Red_UR_y")).intValue();
            island_ll[0] = ((Long) data.get("Island_LL_x")).intValue();
            island_ll[1] = ((Long) data.get("Island_LL_y")).intValue();
            island_ur[0] = ((Long) data.get("Island_UR_x")).intValue();
            island_ur[1] = ((Long) data.get("Island_UR_y")).intValue();
            tunnel_ll[0] = ((Long) data.get("TNR_LL_x")).intValue();
            tunnel_ll[1] = ((Long) data.get("TNR_LL_y")).intValue();
            tunnel_ur[0] = ((Long) data.get("TNR_UR_x")).intValue();
            tunnel_ur[1] = ((Long) data.get("TNR_UR_y")).intValue();
            sz_ll[0] = ((Long) data.get("SZR_LL_x")).intValue();
            sz_ll[1] = ((Long) data.get("SZR_LL_y")).intValue();
            sz_ur[0] = ((Long) data.get("SZR_UR_x")).intValue();
            sz_ur[1] = ((Long) data.get("SZR_UR_y")).intValue();
            startCorner = ((Long) data.get("RedCorner")).intValue();
            //targetColor = ((Long) data.get("GreenTeam")).intValue();
        } catch(Exception e) {
            System.err.println("Error: " + e.getMessage());
        }

        targetColor = 1;
        ID = new Identifier(SCANNER, targetColor, IDRGB, 1000, LCD);

        /*--- Offset coordinates of tunnel as navigation target (so robot doesn't hit the corner of it ---*/
        double[] tunnelLeft = {tunnel_ll[0] - 1, tunnel_ll[1] + .5};
        double[] tunnelRight = {tunnel_ur[0] + 1, tunnel_ur[1] - .5};

        ODO.start();
        NAV.setup(sz_ll, sz_ur, startCorner); // initialize values of lower left and upper right of table and start corner; initialize acceleration, speed, and light sensor baseline
        NAV.localize();
        if(DEBUG) {
            System.out.println("" + DF.format(Odometer.getX()) + ", " + DF.format(Odometer.getY()) + "," + DF.format(Odometer.getT()));
            if(Button.waitForAnyPress() == Button.ID_ESCAPE) System.exit(0);
        }


        NAV.travelTo(tunnelLeft); 
        if(DEBUG) {
            System.out.println("Tunnel reached");
            System.out.println("" + DF.format(Odometer.getX()) + ", " + DF.format(Odometer.getY()) + "," + DF.format(Odometer.getT()));
            if(Button.waitForAnyPress() == Button.ID_ESCAPE) System.exit(0);
        }


        int speed = NAV.getSpeed();
        NAV.setSpeed(Math.min(4 * speed, 800));
        NAV.travelTo(tunnelRight);
        if(DEBUG) {
            System.out.println("Island reached");
            if(Button.waitForAnyPress() == Button.ID_ESCAPE) System.exit(0);
        }
        NAV.setSpeed(speed);

        NAV.reLocalize();
        Odometer.setX(tunnelRight[0]);
        Odometer.setY(tunnelRight[1]);
        if(DEBUG) {
            System.out.println("Relocalized");
            if(Button.waitForAnyPress() == Button.ID_ESCAPE) System.exit(0);
        }


        NAV.travelTo(sz_ll);
        NAV.reLocalize();
        try {
            Thread.sleep(1000);
            Sound.beep();
            Sound.beep();
            Sound.beep();
            Sound.beep();
            Sound.beep();
        } catch(InterruptedException e) {

        }
        if(DEBUG) {
            if(Button.waitForAnyPress() == Button.ID_ESCAPE) System.exit(0);
        }


        //perform search for can
        NAV.search();

        NAV.travelTo(sz_ur);
        Sound.beep();
        Sound.beep();
        Sound.beep();
        Sound.beep();
        Sound.beep();

        //        int buttonChoice = 0;
        //        do {
        //            LCD.drawInt(targetColor, 0, 0);
        //            buttonChoice = Button.waitForAnyPress();
        //            if(buttonChoice == Button.ID_LEFT && targetColor > 1) {
        //                targetColor--;
        //            } else if(buttonChoice == Button.ID_RIGHT && targetColor < 4) {
        //                targetColor++;
        //            } else if(buttonChoice == Button.ID_ENTER) {
        //                ID.setTargetColor(targetColor);
        //                ID.isTargetCan();
        //            } else if(buttonChoice == Button.ID_ESCAPE) {
        //                System.exit(0);
        //            } else if(buttonChoice == Button.ID_UP) {
        //                ID.computeReferences();
        //            }
        //        } while(Button.waitForAnyPress(1) != Button.ID_ESCAPE);
    }

}
