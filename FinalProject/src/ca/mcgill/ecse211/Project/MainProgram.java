package ca.mcgill.ecse211.Project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class MainProgram {

    // ** Set these as appropriate for your team and current situation **
    //private static final String SERVER_IP = "192.168.2.55";
    private static final int TEAM_NUMBER = 1;

    // Enable/disable printing of debug info from the WiFi class
    private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
    
    private static final EV3MediumRegulatedMotor SCANNER = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
    private static final SampleProvider IDRGB = new EV3ColorSensor(LocalEV3.get().getPort("S4")).getRGBMode();
    private static final TextLCD LCD = LocalEV3.get().getTextLCD();
    
    private static final String LEFT_MOTOR = "B";
    private static final String RIGHT_MOTOR = "A";
    private static final String Claw = "D";
    private static final Navigation NAV = Navigation.getNavigation(LEFT_MOTOR, RIGHT_MOTOR);
    private static final Odometer ODO = new Odometer();
    private static Identifier ID;
    
    private static final ClawController CLAW = new ClawController(Claw);

    @SuppressWarnings("rawtypes")
    public static void main(String[] args) {
    	
    	/* Testing Code Block */
    	ID = new Identifier(SCANNER, 3, IDRGB, 10, LCD);
    	while(true) {
    		CLAW.close();
    		try {Thread.sleep(5000);} catch (InterruptedException e) {}
    		ID.isTargetCan();
    		CLAW.open();
    		try {Thread.sleep(5000);} catch (InterruptedException e) {}
    	}
    	/*
    	ODO.start();
    	NAV.localize();
    	NAV.travelTo(4, 2);
    	NAV.turnTo(0);
    	NAV.reLocalize();
    	NAV.travelTo(4, 2.5);
    	NAV.travelTo(7, 2.5);
    	NAV.travelTo(7, 1);
    	NAV.travelTo(1, 1);
    	NAV.turnTo(0);
    	NAV.reLocalize();
    	*/
    	//NAV.turnLeft(90);
    	/*
    	while(count<10) {
    		
    		NAV.turnLeft(90);
    		try {Thread.sleep(100);} catch (InterruptedException e) {}
    		
    		NAV.turnRight(90);
    		try {Thread.sleefxp(100);} catch (InterruptedException e) {}
    		NAV.turnRight(90);
    		try {Thread.sleep(100);} catch (InterruptedException e) {}
    		NAV.turnLeft(90);
    		try {Thread.sleep(100);} catch (InterruptedException e) {}
    		
    	}
    	*/
    	/* Testing Code Block */
    	
    	/*
        // Initialize WifiConnection class
        WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
        
        int[] ll = new int[2];
        int[] ur = new int[2];
        int[] island_ll = new int[2];
        int[] island_ur = new int[2];
        int[] tunnel_ll = new int[2];
        int[] tunnel_ur = new int[2];
        int[] sz_ll = new int[2];
        int[] sz_ur = new int[2];
        int startCorner = 0, targetColor = 0;
        
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
            tunnel_ll[0] = ((Long) data.get("TNR_LL_x")).intValue() - 1;
            tunnel_ll[1] = ((Long) data.get("TNR_LL_y")).intValue();
            tunnel_ur[0] = ((Long) data.get("TNR_UR_x")).intValue() + 1;
            tunnel_ur[1] = ((Long) data.get("TNR_UR_y")).intValue();
            sz_ll[0] = ((Long) data.get("SZR_LL_x")).intValue();
            sz_ll[1] = ((Long) data.get("SZR_LL_y")).intValue();
            sz_ur[0] = ((Long) data.get("SZR_UR_x")).intValue();
            sz_ur[1] = ((Long) data.get("SZR_UR_y")).intValue();
            startCorner = ((Long) data.get("RedCorner")).intValue();
            targetColor = ((Long) data.get("GreenTeam")).intValue();
        } catch(Exception e) {
            System.err.println("Error: " + e.getMessage());
        }
        
        ID = new Identifier(SCANNER, targetColor, IDRGB, 1000, LCD);
        ODO.start();
        NAV.setup(ll, ur, startCorner);
        NAV.localize();
        NAV.travelTo(tunnel_ll);
        NAV.travelTo(tunnel_ll[0], tunnel_ll[1] + (tunnel_ur[1] - tunnel_ll[1])/2);
        NAV.travelTo(tunnel_ur[0], tunnel_ur[1] - (tunnel_ur[1] - tunnel_ll[1]/2));
        NAV.travelTo(sz_ll);
        try {
            Thread.sleep(1000);
            LocalEV3.get().getAudio().systemSound(3);
        } catch(InterruptedException e) {
            
        }
        
        //perform search for can
        NAV.travelTo(sz_ur);
        LocalEV3.get().getAudio().systemSound(3);
        */
    }

}