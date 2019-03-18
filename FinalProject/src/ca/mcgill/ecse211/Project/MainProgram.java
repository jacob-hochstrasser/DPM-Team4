package ca.mcgill.ecse211.Project;

import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.ev3.LocalEV3;

public class MainProgram {

    // ** Set these as appropriate for your team and current situation **
    private static final String SERVER_IP = "192.168.2.25";
    private static final int TEAM_NUMBER = 1;

    // Enable/disable printing of debug info from the WiFi class
    private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
    
    private static final String LEFT_MOTOR = "B";
    private static final String RIGHT_MOTOR = "D";
    private static final Navigation NAV = Navigation.getNavigation(LEFT_MOTOR, RIGHT_MOTOR);

    public static void main(String[] args) {

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
            ll[0] = ((Long) data.get("")).intValue();
            ll[1] = ((Long) data.get("")).intValue();
            ur[0] = ((Long) data.get("")).intValue();
            ur[1] = ((Long) data.get("")).intValue();
            island_ll[0] = ((Long) data.get("")).intValue();
            island_ll[1] = ((Long) data.get("")).intValue();
            island_ur[0] = ((Long) data.get("")).intValue();
            island_ur[1] = ((Long) data.get("")).intValue();
            tunnel_ll[0] = ((Long) data.get("")).intValue() - 1;
            tunnel_ll[1] = ((Long) data.get("")).intValue();
            tunnel_ur[0] = ((Long) data.get("")).intValue() + 1;
            tunnel_ur[1] = ((Long) data.get("")).intValue();
            sz_ll[0] = ((Long) data.get("")).intValue();
            sz_ll[1] = ((Long) data.get("")).intValue();
            sz_ur[0] = ((Long) data.get("")).intValue();
            sz_ur[1] = ((Long) data.get("")).intValue();
            startCorner = ((Long) data.get("")).intValue();
            targetColor = ((Long) data.get("")).intValue();
        } catch(Exception e) {
            System.err.println("Error: " + e.getMessage());
        }
        
        NAV.setup(ll, ur, startCorner);
        NAV.localize();
        NAV.travelTo(tunnel_ll);
        NAV.travelTo(tunnel_ll[0], tunnel_ll[1] + (tunnel_ur[1] - tunnel_ll[1])/2);
        NAV.travelTo(tunnel_ur[0], tunnel_ur[1] - (tunnel_ur[1] - tunnel_ll[1]/2));
        NAV.travelTo(sz_ll);
        LocalEV3.get().getAudio().systemSound(3);
        //perform search for can
        NAV.travelTo(sz_ur);
        LocalEV3.get().getAudio().systemSound(3);
        
    }

}
