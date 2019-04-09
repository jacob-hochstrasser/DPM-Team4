package ca.mcgill.ecse211.Project;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class MainProgram {

    // ** Set these as appropriate for your team and current situation **
    private static final String SERVER_IP = "192.168.2.28";
    private static final int TEAM_NUMBER = 4;

    // Enable/disable printing of debug info from the WiFi class
    private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;

    private static final EV3MediumRegulatedMotor SCANNER = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
    private static final SampleProvider IDRGB = new EV3ColorSensor(LocalEV3.get().getPort("S4")).getRGBMode();
    private static final TextLCD LCD = LocalEV3.get().getTextLCD();

    private static final String LEFT_MOTOR = "B";
    private static final String RIGHT_MOTOR = "A";
    private static final String Claw = "D";
    private static final Navigation NAV = Navigation.getNavigation(LEFT_MOTOR, RIGHT_MOTOR);
    private static final Odometer ODO = new Odometer();
    private static boolean amIRed = false;
    public static final Identifier ID = new Identifier(SCANNER, IDRGB, LCD);

    public static final ClawController CLAW = new ClawController(Claw);

    @SuppressWarnings("rawtypes")
    public static void main(String[] args) {
        ODO.start();
        NAV.reLocalize(1,1);
        NAV.turnTo(315);
        NAV.moveForward(30.28*Math.sqrt(2)/2);
        NAV.turnTo(90);
        NAV.moveForward(3*30.28);
        //identifierTest();
    	//Initialize WifiConnection class
        //        WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
        //
        //        int[] ll = new int[2];
        //        int[] ur = new int[2];
        //        int[] island_ll = new int[2];
        //        int[] island_ur = new int[2];
        //        int[] tunnel_ll = new int[2];
        //        int[] tunnel_ur = new int[2];
        //        int[] sz_ll = new int[2];
        //        int[] sz_ur = new int[2];
        //        int startCorner = 0;
        //        try {
        //            Map data = conn.getData();
        //            amIRed = (TEAM_NUMBER == ((Long) data.get("RedTeam")).intValue());
        //            if(amIRed) {
        //                ll[0] = ((Long) data.get("Red_LL_x")).intValue();
        //                ll[1] = ((Long) data.get("Red_LL_y")).intValue();
        //                ur[0] = ((Long) data.get("Red_UR_x")).intValue();
        //                ur[1] = ((Long) data.get("Red_UR_y")).intValue();
        //                tunnel_ll[0] = ((Long) data.get("TNR_LL_x")).intValue();
        //                tunnel_ll[1] = ((Long) data.get("TNR_LL_y")).intValue();
        //                tunnel_ur[0] = ((Long) data.get("TNR_UR_x")).intValue();
        //                tunnel_ur[1] = ((Long) data.get("TNR_UR_y")).intValue();
        //                sz_ll[0] = ((Long) data.get("SZR_LL_x")).intValue();
        //                sz_ll[1] = ((Long) data.get("SZR_LL_y")).intValue();
        //                sz_ur[0] = ((Long) data.get("SZR_UR_x")).intValue();
        //                sz_ur[1] = ((Long) data.get("SZR_UR_y")).intValue();
        //                startCorner = ((Long) data.get("RedCorner")).intValue();
        //            } else if(!amIRed) {
        //                ll[0] = ((Long) data.get("Green_LL_x")).intValue();
        //                ll[1] = ((Long) data.get("Green_LL_y")).intValue();
        //                ur[0] = ((Long) data.get("Green_UR_x")).intValue();
        //                ur[1] = ((Long) data.get("Green_UR_y")).intValue();
        //                tunnel_ll[0] = ((Long) data.get("TNG_LL_x")).intValue();
        //                tunnel_ll[1] = ((Long) data.get("TNG_LL_y")).intValue();
        //                tunnel_ur[0] = ((Long) data.get("TNG_UR_x")).intValue();
        //                tunnel_ur[1] = ((Long) data.get("TNG_UR_y")).intValue();
        //                sz_ll[0] = ((Long) data.get("SZG_LL_x")).intValue();
        //                sz_ll[1] = ((Long) data.get("SZG_LL_y")).intValue();
        //                sz_ur[0] = ((Long) data.get("SZG_UR_x")).intValue();
        //                sz_ur[1] = ((Long) data.get("SZG_UR_y")).intValue();
        //                startCorner = ((Long) data.get("GreenCorner")).intValue();
        //            }
        //
        //            island_ll[0] = ((Long) data.get("Island_LL_x")).intValue();
        //            island_ll[1] = ((Long) data.get("Island_LL_y")).intValue();
        //            island_ur[0] = ((Long) data.get("Island_UR_x")).intValue();
        //            island_ur[1] = ((Long) data.get("Island_UR_y")).intValue();
        //        } catch(Exception e) {
        //            System.err.println("Error: " + e.getMessage());
        //        }
        //
        //        /*--- Offset coordinates of tunnel as navigation target (so robot doesn't hit the corner of it ---*/
        //        double[] tunnelLeft = {tunnel_ll[0] - 1, tunnel_ll[1] + .5};
        //        double[] tunnelRight = {tunnel_ur[0] + 1, tunnel_ur[1] - .5};
        //        int[] startPos = {ll[0] + 1, ll[1] + 1};
        //
        //        NAV.setup(sz_ll, sz_ur, startCorner, ll, ur, island_ll, island_ur, tunnel_ll, tunnel_ur);
        //        ODO.start();
        //        NAV.localize();
        //        NAV.passTheTunnel(true);
        //        NAV.travelTo(sz_ll);
        //        NAV.reLocalize(sz_ll[0], sz_ll[1]);
        //        NAV.search();
        //        NAV.setSpeed(450);
        //        NAV.setAcceleration(1200);
        //        NAV.turnTo(0);
        //        NAV.reLocalize(sz_ll[0], sz_ll[1]);
        //        NAV.passTheTunnel(false);
        //        if(startCorner == 0) {
        //            NAV.travelTo(1, 1);
        //        } else if(startCorner == 1) {
        //            NAV.travelTo(14, 1);
        //        } else if(startCorner == 2) {
        //            NAV.travelTo(14, 8);
        //        } else {
        //            NAV.travelTo(1, 8);
        //        }
        //
        //        NAV.moveForward(30.28*Math.sqrt(2)/2);
        //
        //        CLAW.open();
        //        for(int i = 0; i < 5; i++) {
        //            Sound.beep();
        //        }
        //        NAV.moveBackward(30.28*Math.sqrt(2)/2);
        //        NAV.turnTo(0);
        //        if(startCorner == 0) {
        //            NAV.reLocalize(1, 1);
        //        } else if(startCorner == 1) {
        //            NAV.reLocalize(14, 1);
        //        } else if(startCorner == 2) {
        //            NAV.reLocalize(14, 8);
        //        } else {
        //            NAV.reLocalize(1, 8);
        //        }
    	
        //    	NAV.travelTo(tunnelLeft[0], tunnel_ll[1]);
        //    	NAV.reLocalize(tunnelLeft[0], tunnel_ll[1]);
        //
        //
        //    	NAV.travelTo(tunnelLeft);
        //    	NAV.travelTo(tunnelRight);
        //    	NAV.reLocalize(tunnelRight[0], tunnel_ur[1]);
        //    	System.out.println(Odometer.getX());
        //    	System.out.println(Odometer.getY());
        //    	System.out.println(Odometer.getT());
        //
        //    	NAV.travelTo(tunnelRight);
        //    	NAV.travelTo(tunnelLeft);
        //    	NAV.reLocalize(tunnelLeft[0], tunnel_ll[1] + 1);
        //
        //    	NAV.travelTo(startPos);
    }

    private static void identifierTest() {
        while(true) {
            CLAW.close();
            try {Thread.sleep(3000);} catch (InterruptedException e) {}
            ID.identifyCan();
            CLAW.open();
            try {Thread.sleep(3000);} catch (InterruptedException e) {}
        }
    }
}