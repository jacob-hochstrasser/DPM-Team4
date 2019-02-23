package ca.mcgill.ecse211.project;

import lejos.hardware.sensor.EV3ColorSensor;

public class Identifier {

    public static enum TARGET_COLOR {
        Any,
        Blue, 
        Green, 
        Yellow, 
        Red;
        
        public static String tcToString(int target) {
            String value;
            switch(target) {
            case 1:
                value = TARGET_COLOR.Blue.toString();
                break;
            case 2:
                value = TARGET_COLOR.Green.toString();
                break;
            case 3:
                value = TARGET_COLOR.Yellow.toString();
                break;
            case 4:
                value = TARGET_COLOR.Red.toString();
                break;
            default:
                value = TARGET_COLOR.Any.toString();
                break;
            }
            return value;
        }
        
        public static TARGET_COLOR decodeValue(int target) {
            TARGET_COLOR value;
            switch(target) {
            case 1:
                value = TARGET_COLOR.Blue;
                break;
            case 2:
                value = TARGET_COLOR.Green;
                break;
            case 3:
                value = TARGET_COLOR.Yellow;
                break;
            case 4:
                value = TARGET_COLOR.Red;
                break;
            default:
                value = TARGET_COLOR.Any;
                break;
            }
            return value;
        }
    }
    
    private final TARGET_COLOR target;
    private final EV3ColorSensor cs;
    
    private static final float[][] LI_REFERENCE_STATS = {{0.25f, 0.25f, 0.75f}, {0.25f, 0.75f, 0.25f}, {0.75f, 0.75f, 0.25f}, {0.75f, 0.25f, 0.25f}};
    
    public Identifier (int target, EV3ColorSensor cs) {
        this.target = TARGET_COLOR.decodeValue(target);
        this.cs = cs;
    }
}
