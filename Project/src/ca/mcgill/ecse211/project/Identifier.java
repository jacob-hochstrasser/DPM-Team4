package ca.mcgill.ecse211.project;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class Identifier {

    /*<-------------------------------- Static Variables ------------------------------------------>*/
    /*<------------------------------------ Public ------------------------------------------------>*/
    /**
     * An enumeration of the different target colors that may be chosen. Includes methods for return a string 
     * of the name of the option chosen and for decoding an integer value to return the corresponding option.
     * @author jacob
     *
     */
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
    /**
     * The distance the robot stops from the can to start scanning it
     */
    public static final float SCANNING_DISTANCE = 2.5f;
    
    /*<----------------------------------- Private ------------------------------------------------->*/
    /**
     * 
     */
    private static final float[][] LI_REFERENCE_STATS = {{0.25f, 0.25f, 0.75f}, {0.25f, 0.75f, 0.25f}, {0.75f, 0.75f, 0.25f}, {0.75f, 0.25f, 0.25f}};
    
    /*<------------------------------------ Instance variables ------------------------------------->*/
    private final TARGET_COLOR target;
    private final SampleProvider idLSRGB;
    private float[] idLSData;
    private int numSamples;
    private boolean isSampling;
    private float [][] samples;
    
    /**
     * Creates an Identifier object with the specified parameters
     * @param target - The target color {1 - Blue, 2 - Green, 3 - Yellow, 4 - Red}
     * @param idLSRGB - The sample provider for the light sensor (RGB mode)
     * @param numSamples - The number of samples taken when scanning
     */
    public Identifier (int target, SampleProvider idLSRGB, int numSamples) {
        this.target = TARGET_COLOR.decodeValue(target);
        this.idLSRGB = idLSRGB;
        if(numSamples%4 == 0) {
            this.numSamples = numSamples;
        } else {
            this.numSamples = 8;
        }
        this.isSampling = false;
        this.idLSData = new float[idLSRGB.sampleSize()];
        samples = new float[numSamples][3];
    }
    
    /**
     * Scans the current can a determines if it is the target can or not
     * @return boolean representing whether a target was found
     */
    public boolean isTargetCan() {
        this.isSampling = true;
        for(int i = 0; i < numSamples; i++) {
            idLSRGB.fetchSample(idLSData, 0);
            
        }
        return true;
    }
    
    /**
     * Changes the number of samples taken during scanning
     * @param newNumSamples
     */
    public void setNumSamples(int newNumSamples) {
        if(!isSampling &&(newNumSamples%4 == 0)) {
            this.numSamples = newNumSamples;
            samples = new float[numSamples][3];
        }
    }
}
