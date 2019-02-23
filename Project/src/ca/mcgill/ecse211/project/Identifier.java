package ca.mcgill.ecse211.project;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

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
    
    public static final float samplingDistance = 2.5f;
    private final TARGET_COLOR target;
    private final SampleProvider idLSRGB;
    private float[] idLSData;
    private int numSamples;
    private boolean isSampling;
    
    private static final float[][] LI_REFERENCE_STATS = {{0.25f, 0.25f, 0.75f}, {0.25f, 0.75f, 0.25f}, {0.75f, 0.75f, 0.25f}, {0.75f, 0.25f, 0.25f}};
    private float [][] samples;
    
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
    
//    public void run() {
//        
//    }
    
    public boolean identifyCan() {
        this.isSampling = true;
        for(int i = 0; i < numSamples; i++) {
            idLSRGB.fetchSample(idLSData, 0);
            
        }
        return true;
    }
    public void setNumSamples(int newNumSamples) {
        if(!isSampling &&(newNumSamples%4 == 0)) {
            this.numSamples = newNumSamples;
            samples = new float[numSamples][3];
        }
    }
}
