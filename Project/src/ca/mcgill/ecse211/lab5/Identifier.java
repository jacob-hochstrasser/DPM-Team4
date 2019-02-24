package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
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
     * Normalized mean RGB values of the four can colors
     */
    private static final float[][] LI_REFERENCE_MEANS = {{0.5300081f, 0.70231813f, 0.475227f},
            {0.53296834f, 0.6995804f, 0.47595385f},
            {0.5332132f, 0.70027846f, 0.47465125f},
            {0.5293638f, 0.7022591f, 0.47603157f}};
    /**
     * Standard deviation of RGB samples of four can colors
     */
    private static final float[][] LI_REFERENCE_SDS = {{0.3569843f, 0.4730426f, 0.32008716f},
            {0.357979f, 0.46988732f, 0.31968448f},
            {0.35736787f, 0.4693373f, 0.31811908f},
            {0.35569653f, 0.4718699f, 0.3198611f}};
    /**
     * Scanning motor angular acceleration
     */
    private static final int ACCELERATION = 1000;
    /**
     * Scanning motor angular speed
     */
    private static final int SPEED = 140;
    
    
    /*<------------------------------------ Instance variables ------------------------------------->*/
    private final EV3MediumRegulatedMotor scanner;
    private final TARGET_COLOR target;
    private final int targetInt;
    private final SampleProvider idLSRGB;
    private int numSamples;
    private boolean isSampling;
    private float [][] samples;
    private float [] sampleMeans = new float[3];
    private float [] sampleErrors = new float[3];
    
    /**
     * Creates an Identifier object with the specified parameters
     * @param target - The target color {1 - Blue, 2 - Green, 3 - Yellow, 4 - Red}
     * @param idLSRGB - The sample provider for the light sensor (RGB mode)
     * @param numSamples - The number of samples taken when scanning
     */
    public Identifier (EV3MediumRegulatedMotor scanner, int target, SampleProvider idLSRGB, int numSamples) {
        this.scanner = scanner;
        this.target = TARGET_COLOR.decodeValue(target);
        this.targetInt = target;
        this.idLSRGB = idLSRGB;
        if(numSamples >= 0) {
            this.numSamples = numSamples;
        } else {
            this.numSamples = 100;
        }
        this.isSampling = false;
        samples = new float[numSamples][idLSRGB.sampleSize()];
        
        scanner.setAcceleration(ACCELERATION);
        scanner.setSpeed(SPEED);
        
    }
    
    /**
     * Scans the current can a determines if it is the target can or not
     * @return boolean representing whether a target was found
     */
    public boolean isTargetCan() {
        this.isSampling = true;
        (new Thread() {
            public void run() {
                scanner.rotate(280, false);
                scanner.rotate(-280, false);
            }
        }).start();
        for(int i = 0; i < samples.length; i++) {
            idLSRGB.fetchSample(samples[i], 0);
            try {
                // sampling rate depends on sample size
                Thread.sleep(40);  
            } catch(InterruptedException e) {
                e.printStackTrace();
            } 
        }
        computeNormalizedMeans(samples, sampleMeans);
        computeErrors(sampleMeans, sampleErrors, targetInt);
        if((Math.abs(sampleErrors[0]) < 2 * LI_REFERENCE_SDS[targetInt][0]) && (Math.abs(sampleErrors[1]) < 2 * LI_REFERENCE_SDS[targetInt][1]) && (Math.abs(sampleErrors[2]) < 2 * LI_REFERENCE_SDS[targetInt][2])) {
            return true; 
        } else {
            return false;
        }
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
    
    public void myMethod() {
        for(int i=0; i< samples.length; i++) {
            idLSRGB.fetchSample(samples[i], 0);
            try {
                Thread.sleep(40);  
            } catch(InterruptedException e) {
                e.printStackTrace();
            } 
        }
        
        
    }
    
    private static void computeNormalizedMeans(float[][] samples, float[] means) {
        float sumR = 0, sumG = 0, sumB = 0;
        for(float[] f : samples) {
            sumR += f[0];
            sumG += f[1];
            sumB += f[2];
        }
        
        float normalizedR = sumR/(float)samples.length;
        float normalizedG = sumG/(float)samples.length;
        float normalizedB = sumB/(float)samples.length;
        
        normalizedR = (float) (normalizedR/Math.sqrt(Math.pow(normalizedR, 2) + Math.pow(normalizedG, 2) + Math.pow(normalizedB, 2)));
        normalizedG = (float) (normalizedG/Math.sqrt(Math.pow(normalizedR, 2) + Math.pow(normalizedG, 2) + Math.pow(normalizedB, 2)));
        normalizedB = (float) (normalizedB/Math.sqrt(Math.pow(normalizedR, 2) + Math.pow(normalizedG, 2) + Math.pow(normalizedB, 2)));
        
        means[0] = normalizedR;
        means[1] = normalizedG;
        means[2] = normalizedB;
    }
    
    private static void computeErrors(float[] data, float[] errors, int index) {
        errors[0] = data[0] - LI_REFERENCE_MEANS[index][0];
        errors[1] = data[1] - LI_REFERENCE_MEANS[index][1];
        errors[2] = data[2] - LI_REFERENCE_MEANS[index][2];
        
    }
    
    private static void computeStdDev(float[][] samples, float[] means, float [] sds) {
        float intermR = 0, intermG = 0, intermB = 0;
        for(float[] f : samples) {
            intermR += (Math.pow(f[0] - means[0], 2));
            intermG += (Math.pow(f[1] - means[1], 2));
            intermB += (Math.pow(f[2] - means[2], 2));
        }
        
        sds[0] = (float)Math.sqrt(intermR/(samples.length-1));
        sds[1] = (float)Math.sqrt(intermG/(samples.length-1));
        sds[2] = (float)Math.sqrt(intermB/(samples.length-1));  
    }
}
