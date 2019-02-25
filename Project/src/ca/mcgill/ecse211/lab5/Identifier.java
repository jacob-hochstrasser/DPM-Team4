package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
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
    public static final float SCANNING_DISTANCE = 5.0f;
    
    /*<----------------------------------- Private ------------------------------------------------->*/
    /**
     * Normalized mean RGB values of the four can colors
     */
    private static final float[][] LI_REFERENCE_MEANS = {{0.35362998f, 0.2969332f, 0.10991809f}, 
            {0.2804255f, 0.18570383f, 0.04182702f}, 
            {0.71931106f, 0.11860263f, 0.03738072f}, 
            {0.8550641f, 0.10356925f, 0.044857424f}};
    /**
     * Standard deviation of RGB samples of four can colors
     */
    private static final float[][] LI_REFERENCE_SDS = {{0.031516474f, 0.038391143f, 0.01112207f}, 
            {0.010355728f, 0.025849136f, 0.010378962f}, 
            {0.02544661f, 0.018257428f, 0.012972444f}, 
            {0.06890812f, 0.04407912f, 0.017151868f}};
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
    private TextLCD lcd;
    
    /**
     * Creates an Identifier object with the specified parameters
     * @param target - The target color {1 - Blue, 2 - Green, 3 - Yellow, 4 - Red}
     * @param idLSRGB - The sample provider for the light sensor (RGB mode)
     * @param numSamples - The number of samples taken when scanning
     */
    public Identifier (EV3MediumRegulatedMotor scanner, int target, SampleProvider idLSRGB, int numSamples, TextLCD lcd) {
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
        this.lcd = lcd;
        
        scanner.setAcceleration(ACCELERATION);
        scanner.setSpeed(SPEED);
        
    }
    
    public void idColor() {
        scanCan();
        //        idLSRGB.fetchSample(samples[0], 0);
        //        normalize(samples[0]);
        //        for(int i = 0; i < 3; i++) {
        //            if(within2SDS(samples[0], LI_REFERENCE_MEANS[i], LI_REFERENCE_SDS[i])){
        //                lcd.drawString("Object detected", 0, 0);
        //                lcd.drawString(TARGET_COLOR.tcToString(i), 0, 2);
        //                break;
        //            }
        //        }
        try {
            Thread.sleep(1000);
        } catch(InterruptedException e) {
            
        }
        
        //        for(float f : samples[0]) {
        //            System.out.print(f + ", ");
        //        }
        //        System.out.print("\n");
        //        for(float f : sampleErrors) {
        //            System.out.print(f + ", ");
        //        }
        //        System.out.print("\n");
        //        try {
        //            Thread.sleep(2000);
        //        } catch(InterruptedException e) {
        //        }
        //        
        //        for(int i=1; i < LI_REFERENCE_MEANS.length; i++) {
        //            if((Math.abs(sampleErrors[0]) < 2 * LI_REFERENCE_SDS[i][0]) && (Math.abs(sampleErrors[1]) < 2 * LI_REFERENCE_SDS[i][1]) && (Math.abs(sampleErrors[2]) < 2 * LI_REFERENCE_SDS[i][2])) {
        //                lcd.drawString("Object found", 0, 0);
        //                lcd.drawString(TARGET_COLOR.tcToString(i), 0, 2);
        //            } else {
        //                lcd.clear();
        //            }
        //        }
    }
    /**
     * Scans the current can a determines if it is the target can or not
     * @return boolean representing whether a target was found
     */
    public boolean isTargetCan() {
        //        lcd.drawString("Object found", 0, 0);
        //        this.isSampling = true;
        scanCan();
        computeMeans(samples, sampleMeans);
        lcd.drawString(sampleMeans[0] + ", " + sampleMeans[1] + ", " + sampleMeans[2], 0, 4);
        
        //computeErrors(sampleMeans, sampleErrors, targetInt);
        //        if((Math.abs(sampleErrors[0]) < 2 * LI_REFERENCE_SDS[targetInt][0]) && (Math.abs(sampleErrors[1]) < 2 * LI_REFERENCE_SDS[targetInt][1]) && (Math.abs(sampleErrors[2]) < 2 * LI_REFERENCE_SDS[targetInt][2])) {
        //            lcd.drawString(TARGET_COLOR.tcToString(targetInt), 0, 1);
        //            LocalEV3.get().getAudio().systemSound(0);
        //            try {
        //                Thread.sleep(2000);
        //            } catch(InterruptedException e) {
        //                e.printStackTrace();
        //            }
        //            this.isSampling = false;
        //            return true; 
        //        } else {
        //            for(int i=1; i < LI_REFERENCE_MEANS.length; i++) {
        //                if(i != targetInt) {
        //                    if((Math.abs(sampleErrors[0]) < 2 * LI_REFERENCE_SDS[i][0]) && (Math.abs(sampleErrors[1]) < 2 * LI_REFERENCE_SDS[i][1]) && (Math.abs(sampleErrors[2]) < 2 * LI_REFERENCE_SDS[i][2])) {
        //                        lcd.drawString(TARGET_COLOR.tcToString(i), 0, 1);
        //                    }
        //                }
        //            }
        //            LocalEV3.get().getAudio().systemSound(1);
        //            try {
        //                Thread.sleep(2000);
        //            } catch(InterruptedException e) {
        //                e.printStackTrace();
        //            }
        //            this.isSampling = false;
        //            return false;
        //        }
    }

    private void scanCan() {
        (new Thread() {
            public void run() {
                scanner.rotate(180, false);
                scanner.rotate(-180, false);
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
    
    public static void computeMeans(float[][] samples, float[] means) {
        float sumR = 0, sumG = 0, sumB = 0;
        for(float[] f : samples) {
            sumR += f[0];
            sumG += f[1];
            sumB += f[2];
        }
        
        float meanR = sumR/(float)samples.length;
        float meanG = sumG/(float)samples.length;
        float meanB = sumB/(float)samples.length;
        
        means[0] = meanR;
        means[1] = meanG;
        means[2] = meanB;
    }
    
    public static void normalize(float[] data) {
        data[0] = (float) (data[0]/Math.sqrt(Math.pow(data[0], 2) + Math.pow(data[1], 2) + Math.pow(data[2], 2)));
        data[1] = (float) (data[1]/Math.sqrt(Math.pow(data[0], 2) + Math.pow(data[1], 2) + Math.pow(data[2], 2)));
        data[2] = (float) (data[2]/Math.sqrt(Math.pow(data[0], 2) + Math.pow(data[1], 2) + Math.pow(data[2], 2)));
    }
    
    private static void computeErrors(float[] data, float[] errors, int index) {
        errors[0] = data[0] - LI_REFERENCE_MEANS[index][0];
        errors[1] = data[1] - LI_REFERENCE_MEANS[index][1];
        errors[2] = data[2] - LI_REFERENCE_MEANS[index][2];
        
    }
    
    public static void computeStdDev(float[][] samples, float[] means, float [] sds) {
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
    
    private static boolean within2SDS(float[] data, float[] ref, float[] sds) {
        float[] result = new float[3];
        result[0] = data[0] - ref[0];
        result[1] = data[1] - ref[1];
        result[2] = data[2] - ref[2];
        if(Math.abs(result[0]) <= 2 * sds[0] && Math.abs(result[1]) <= 2 * sds[1] && Math.abs(result[2]) <= 2 * sds[2]){
            return true;
        } else {
            return false;
        }
    }
    
    private static float computeEuclidianDistance(float[] data, float[] ref) {
        float[] resultArray = new float[3];
        resultArray[0] = (float)Math.pow(data[0] - ref[0], 2);
        resultArray[1] = (float)Math.pow(data[1] - ref[1], 2);
        resultArray[2] = (float)Math.pow(data[2] - ref[2], 2);
        float result = (float)Math.sqrt(resultArray[0] + resultArray[1] + resultArray[2]);
        return result;
    }
}
