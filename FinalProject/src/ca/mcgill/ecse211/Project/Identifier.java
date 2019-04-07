package ca.mcgill.ecse211.Project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import java.util.Arrays;

/**
 * This class is responsible for all functions relating to can color identification and searching for the target can
 * 
 * @author Jake
 *
 */
public class Identifier {

    /**
     * An enumeration of the different target colors that may be chosen. Includes methods for return a string 
     * of the name of the option chosen and for decoding an integer value to return the corresponding option.
     * @author Jake
     *
     */
    public static enum TARGET_COLOR {
        Any,
        Blue, 
        Green, 
        Yellow, 
        Red;

        /**
         * Gives the string value of the enum value corresponding to the target index
         * @param target index of enum constant
         * @return string name of corresponding enum constant
         */
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

        /**
         * Returns the enum value corresponding to the target index
         * @param target index of enum constant
         * @return enum constant corresponding to the target
         */
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

    /**
     * Normalized mean RGB values of the four can colors (Blue, Green, Yellow, Red)
     */
    private static final float[][] LI_REFERENCE_MEANS =
        {{0.25196862f, 0.29015315f, 0.14226039f}, {0.18914916f, 0.4638317f, 0.033236276f}, 
            {0.8633784f, 0.054316904f, 0.011821673f}, {0.92697716f, 0.050970152f, 0.024029283f}}; // Y  {0.72614944f, 0.1204941f, 0.026043372f}
    
    /**
     * Standard deviation of RGB samples of four can colors
     */
    private static final float[][] LI_REFERENCE_SDS = 
        {{0.03463795f, 0.04026307f, 0.0149934925f}, {0.042265132f, 0.072264396f, 0.006848849f}, 
            {0.02798699f, 0.023513217f, 0.0053855744f}, {0.04179209f, 0.023005877f, 0.011273368f}}; // Y {0.03653974f, 0.026533125f, 0.016543025f}
    
    /**
     * Scanning motor angular acceleration
     */
    private static final int ACCELERATION = 1000;
    
    /**
     * Scanning motor angular speed
     */
    private static final int SPEED = 90;

    /**
     * Scanning motor that moves the light sensor around the can during color identification
     */
    private final EV3MediumRegulatedMotor scanner;
    
    /**
     * The target color being searched for
     */
    private TARGET_COLOR target;
    
    /**
     * Integer representation of the target color
     */
    private int targetInt;
    
    /**
     * Sample provider for the color sensor
     */
    private final SampleProvider idLSRGB;
    
    /**
     * Number of samples taken during a scan
     */
    private int numSamples;
    
    /**
     * If the Identifier is sampling, makes sure it is not interrupted by other processes
     */
    private boolean isSampling;
    
    /**
     * Stores samples taken during a scan
     */
    private float [][] samples;
    
    /**
     * Stores the mean of the sample distribution
     */
    private float [] sampleMeans = new float[3];
    
    /**
     * Stores the standard deviation of the sample distribution
     */
    private float [] sampleSDS = new float[3];
    
    /**
     * The text lcd of the EV3 brick
     */
    private TextLCD lcd;
    
    //private static Identifier ID;
    
    //    public static Identifier getIdentifier(EV3MediumRegulatedMotor scanner, SampleProvider idLSRGB, TextLCD lcd) {
    //        if(ID == null) {
    //            ID = new Identifier(scanner, idLSRGB, lcd);
    //        }
    //        return ID;
    //    }

    /**
     * Creates an Identifier object with the specified parameters.
     * 
     * @param target The target color {1 - Blue, 2 - Green, 3 - Yellow, 4 - Red}
     * @param idLSRGB The sample provider for the light sensor (RGB mode)
     * @param numSamples The number of samples taken when scanning
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
    
    public Identifier(EV3MediumRegulatedMotor scanner, SampleProvider idLSRGB, TextLCD lcd) {
        this.scanner = scanner;
        this.targetInt = 0;
        this.target = TARGET_COLOR.decodeValue(targetInt);
        this.idLSRGB = idLSRGB;
        this.numSamples = 1000;
        this.isSampling = false;
        samples = new float[numSamples][idLSRGB.sampleSize()];
        this.lcd = lcd;
        
        scanner.setAcceleration(ACCELERATION);
        scanner.setSpeed(SPEED);
    }
    
    /**
     * Scans a single spot on a can and prints which color is being detected.
     */
    public void idColor() {
        this.setNumSamples(100);
        scanCan(false);
        computeMeans(samples, sampleMeans);
        
        // allows us to display nothing on the screen if no objects are held right by the light sensor
        for(float f : sampleMeans) {
            Float f2 = new Float(f);
            if(f2.isNaN()) {
                lcd.clear();
            }
        }
        
        int match = findMatch();
        if(match != -1) {
            lcd.clear();
            lcd.drawString("Object detected", 0, 0);
            lcd.drawString(TARGET_COLOR.tcToString(match), 0, 1);
        }
    }
    
    /**
     * Scans the current can and determines if it is the target can or not.
     * 
     * @return boolean representing whether a target was found
     */
    public boolean isTargetCan() {
        takeSamples();
        int match = findMatch();

        if((match) == targetInt) {
            LocalEV3.get().getAudio().systemSound(0);
            this.isSampling = false;
            return true;
        } else {
            LocalEV3.get().getAudio().systemSound(1);
            this.isSampling = false;
            return false;
        } 
    }
    
    public int identifyCan() {
        takeSamples();
        int match = findMatch();
        if(match == -1) {
            return match;
        }
        
        beep(match, Navigation.weight());
        return match;
    }
    
    private void beep(int numBeeps, boolean isHeavy) {
        for(int i = 0; i < numBeeps; i++) {
            if(isHeavy) {
            	Sound.playTone(659, 1000);
            } else {
            	Sound.playTone(659, 500);
            }
        }
    }
   
    
    private void takeSamples() {
        this.setNumSamples(1000);
        this.isSampling = true;
        scanCan(true);
        computeMeans(samples, sampleMeans);
    }
    
    /**
     * Conducts a scan of a can (assuming there is a can in the scan zone). The scanner can either remain stationary or move around the can during a scan.
     * 
     * @param sweep if true, the scanner moves around the can when sampling
     */
    private void scanCan(boolean sweep) {
        if(sweep){
            (new Thread() {
                public void run() {
                    scanner.rotate(180, false);
                    scanner.rotate(-180, false);
                    scanner.stop();
                }
            }).start();

            for(int i = 0; i < samples.length; i++) {
                idLSRGB.fetchSample(samples[i], 0);
                normalize(samples[i]);
                try {
                    // sampling rate depends on sample size and scanner speed
                    Thread.sleep(4000/numSamples);  
                } catch(InterruptedException e) {
                    e.printStackTrace();
                } 
            }
        } else {
            // filters out null data to prevent erroneous readings
            for(int i = 0; i < samples.length; i++) {
                idLSRGB.fetchSample(samples[i], 0);
                for(float f : samples[i]) {
                    if(f < .0001) {
                        f = 0;
                    }
                }
                normalize(samples[i]);
            }
        }
    }

    public void setTargetColor(int newTarget) {
        if(!isSampling) {
            this.target = TARGET_COLOR.decodeValue(newTarget);
            this.targetInt = newTarget;
        }
    }

    /**
     * Changes the number of samples taken during scanning. Can only happen if a scan is not already in progress.
     * 
     * @param newNumSamples the number of samples to be taken during a scan
     */
    public void setNumSamples(int newNumSamples) {
        if(!isSampling &&(newNumSamples%4 == 0)) {
            this.numSamples = newNumSamples;
            samples = new float[numSamples][3];
        }
    }
    
    /**
     * Determines the most likely color match of a sample of rgb channel values.
     * 
     * @return index of the most likely color match offset by -1
     */
    private int findMatch() {
        float[] errors = new float[LI_REFERENCE_MEANS.length];
        for(int i = 0; i < errors.length; i++) {
            errors[i] = computeStdDevsAway(sampleMeans, LI_REFERENCE_MEANS[i], LI_REFERENCE_SDS[i]);
        }
        
        // algorithm for determining the reference color from which the samples are the fewest standard deviations away
        float temp = 10000;
        int j = 0;
        for(int i = 0; i < errors.length; i++) {
            if(errors[i] < temp) {
                temp = errors[i];
                j = i;
            } 
        }
        
        if(errors[j] <= 9) {
            return j + 1; 
        } else {
            return -1;
        }
        
    }
    
    /**
     * Used to compute the baseline reference values of colored cans to be used later for comparison.
     */
    public void computeReferences() {
        scanCan(true);
        computeMeans(samples, sampleMeans);
        computeStdDevs(samples, sampleMeans, sampleSDS);
        
        // formatted output to console for manual transcription of data
        System.out.print("{");
        for(float f : sampleMeans) {
            System.out.print(f + ", ");
        }
        System.out.println("}");
        System.out.println();
        System.out.print("{");
        for(float f : sampleSDS) {
            System.out.print(f + ", ");
        }
        System.out.print("}");
        System.out.println();
    }
    
    /**
     * Used to compute the arithmetic sample means of the specified samples.
     * 
     * @param samples the sampled rgb values
     * @param means the array in which to store the means
     */
    private static void computeMeans(float[][] samples, float[] means) {
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
    
    /**
     * Normalizes the input data.
     * 
     * @param input vector to be normalized
     */
    public static void normalize(float[] data) {
        data[0] = (float) (data[0]/Math.sqrt(Math.pow(data[0], 2) + Math.pow(data[1], 2) + Math.pow(data[2], 2)));
        data[1] = (float) (data[1]/Math.sqrt(Math.pow(data[0], 2) + Math.pow(data[1], 2) + Math.pow(data[2], 2)));
        data[2] = (float) (data[2]/Math.sqrt(Math.pow(data[0], 2) + Math.pow(data[1], 2) + Math.pow(data[2], 2)));
    }
    
    /**
     * Computes the difference between the input data and the reference means.
     * 
     * @param data the input data
     * @param errors the array to store the error values in
     * @param index determines which reference color to use for comparison with the input data
     */
    private static void computeErrors(float[] data, float[] errors, int index) {
        errors[0] = data[0] - LI_REFERENCE_MEANS[index][0];
        errors[1] = data[1] - LI_REFERENCE_MEANS[index][1];
        errors[2] = data[2] - LI_REFERENCE_MEANS[index][2];
    }
    
    /**
     * Computes the sample standard deviation of the input rgb channels.
     * 
     * @param samples the sample data
     * @param means the means of the input data channels
     * @param sds the array in which to store the standard deviation calculation results
     */
    private static void computeStdDevs(float[][] samples, float[] means, float [] sds) {
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
    
    /**
     * Determines whether the given data lies within two standard deviations of the specified reference values.
     * 
     * @param data the input data to compare with the reference
     * @param means the reference means being compared to the input
     * @param sds the reference standard deviations
     * @return boolean representing whether the sample is within two standard deviations of the reference value
     * 
     * @deprecated As of right now, using
     * {@link #computeStdDevsAway(float[], float[], float[])}
     */
    private static boolean within2SDS(float[] data, float[] means, float[] sds) {
        float[] result = new float[3];
        result[0] = data[0] - means[0];
        result[1] = data[1] - means[1];
        result[2] = data[2] - means[2];
        if(Math.abs(result[0]) <= 2 * sds[0] && Math.abs(result[1]) <= 2 * sds[1] && Math.abs(result[2]) <= 2 * sds[2]){
            return true;
        } else {
            return false;
        }
    }
    
    /**
     * Computes how many standard deviations away the given data is from the reference data.
     * 
     * @param data the input data
     * @param means the mean reference values
     * @param sds the standard deviation reference values
     * @return the number of standard deviations the input data is away from the reference mean
     */
    private static float computeStdDevsAway(float[] data, float[] means, float[] sds) {
        float[] numSDS = new float[3];
        numSDS[0] = Math.abs(data[0] - means[0])/sds[0];
        numSDS[1] = Math.abs(data[1] - means[1])/sds[1];
        numSDS[2] = Math.abs(data[2] - means[2])/sds[2];
        float result = numSDS[0] + numSDS[1] + numSDS[2];
        System.out.println(result);
        return result;
    }
    
    /**
     * Computes the euclidian distance between the input data and the reference data.
     * 
     * @param data the input data
     * @param means the reference means
     * @return the euclidian distance of the input data from the reference data
     * 
     * @deprecated No longer used. Results are too inaccurate. Replaced by
     *  {@link #computeStdDevsAway(float[], float[], float[])}
     */
    private static float computeEuclidianDistance(float[] data, float[] means) {
        float[] resultArray = new float[3];
        resultArray[0] = (float)Math.pow(data[0] - means[0], 2);
        resultArray[1] = (float)Math.pow(data[1] - means[1], 2);
        resultArray[2] = (float)Math.pow(data[2] - means[2], 2);
        float result = (float)Math.sqrt(resultArray[0] + resultArray[1] + resultArray[2]);
        return result;
    }
}