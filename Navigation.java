package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
	
	/** This class localizes and navigates the robot, and searches for cans.
	 * @author Pengnan Fan
	 */
	
	//-----<Important Constants>-----//
	private final double LS_DIFF = 9.6;//cm
	private final double LS_WHEEL_DIFF = 13;//cm
	private final int INITIALIZING_SCOPE = 25;
	private final int MEASURING_SCOPE = 5;
	private final int TURNING_SPEED = 100;
	private final int APPROACHING_SPEED = 150;
	private final int NAVIGATING_SPEED = 250;
	private final int SEARCHING_SPEED = 200;
	private final int ACCELERATION = 500;
	private final int TIME_INTERVAL = 25;//ms
	private final double TILE_SIZE = 30.48;//cm, length of one side of a tile
	private final double STANDARD_DISTANCE_FALLING = TILE_SIZE;//cm, standard distance to distinguish falling edge
	private final double NOTICE_MARGIN = 1;
	private final double ROTATION_ERROR_CW = 30;
	private final double ROTATION_ERROR_CCW = 15;
	private double TRACK = Project.TRACK;
	private double RADIUS = Project.RADIUS;
	private float BLACK_LINE_LEFT;
	private float BLACK_LINE_RIGHT;
	private int START_CORNER;// 0 -> down left, 1 -> down right, 2 -> up right, 3 -> up left
	private int[] LL;
	private int[] UR;
	private int[] position = new int[] {0,0,0};
	
	//-----<Motors>-----//
	private static EV3LargeRegulatedMotor LEFT_MOTOR = Project.LEFT_MOTOR;
	private static EV3LargeRegulatedMotor RIGHT_MOTOR = Project.RIGHT_MOTOR;
	
	//-----<SensorPoller>-----//
	private static SensorPoller LOCALIZING_LEFT = Project.LOCALIZING_LEFT;
	private static SensorPoller LOCALIZING_RIGHT = Project.LOCALIZING_RIGHT;
	private static SensorPoller ULTRASONIC = Project.ULTRASONIC;
	
	/**
	 * Creates a Navigation object which helps to localize, navigate, and search for cans.
	 * 
	 * @param START_CORNER - Integer, 0 -> Down left, 1 -> Down right, 2 -> Up right, 3 -> Up left
	 * @param LL - Integer array suggests the bottom left coordinates of the searching area
	 * @param UR - Integer array suggests the upper right coordinates of the seaching area
	 */
	public Navigation(int START_CORNER, int[] LL, int[] UR) {
		this.START_CORNER = START_CORNER;
		this.LL = LL;
		this.UR = UR;
		Sound.setVolume(100);
	}
	
	private void initialize() {
		float sum_left = 0;
		float sum_right = 0;
		for(int i = 0; i<INITIALIZING_SCOPE; i++) {
			sum_left+=LOCALIZING_LEFT.getData();
			sum_right+=LOCALIZING_RIGHT.getData();
		}
		BLACK_LINE_LEFT = sum_left/INITIALIZING_SCOPE;
		BLACK_LINE_RIGHT = sum_right/INITIALIZING_SCOPE;
	}
	
	private void localize() {
		initialize();
		LEFT_MOTOR.setSpeed(TURNING_SPEED);
		RIGHT_MOTOR.setSpeed(TURNING_SPEED);
		LEFT_MOTOR.setAcceleration(ACCELERATION);
		RIGHT_MOTOR.setAcceleration(ACCELERATION);
		
		if (START_CORNER == 0) {
			//Down left
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			
			double[] pos1 = new double[] {0,0,0};
			boolean isDetected = false;
			float d1 = ULTRASONIC.getData();
			float d2 = 0;
			
			// Rotate clockwise until you detect a falling edge
		    while(!isDetected) {
		    	try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.forward();
		        RIGHT_MOTOR.backward();
		        d2 = d1;
		        d1 = ULTRASONIC.getData();
		        if((d2 >= (STANDARD_DISTANCE_FALLING + NOTICE_MARGIN))&&(d1<= (STANDARD_DISTANCE_FALLING - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos1 = Odometer.getPosition();
		        }
		    }
		    Sound.beep();
		    
		    double[] pos2 = new double[] {0,0,0};
		    isDetected = false;
		    
		    d1 = ULTRASONIC.getData();
			d2 = 0;
			
		    // Rotate counter-clockwise until you detect another falling edge
		    while(!isDetected) {
		        try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.backward();
		        RIGHT_MOTOR.forward();
		        d2 = d1;
		        d1 = ULTRASONIC.getData();
		        if((d2 >= (STANDARD_DISTANCE_FALLING + NOTICE_MARGIN))&&(d1<= (STANDARD_DISTANCE_FALLING - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos2 = Odometer.getPosition();
		        }
		    }
		    
		    Sound.beep();
		    stop();
		    
		    // Calculate angle of local maximum based on two detected edges and use it to find 0° 
		    double dTheta = (-225 - 90 + (pos1[2]+pos2[2])/2 + 360)%360;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
		    //Turn to 0
		    
		    turnTo(dTheta);
		    turnRight(ROTATION_ERROR_CW);
		    Odometer.resetTheta();//Reset theta
		    
		    
		    boolean left = false;
		    boolean right = false;
		    double leftDetection = 0;
		    double rightDetection = 0;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()&&!left) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(left) {
			    		stop();
			    	}
			    }
		    } while(!left||!right);
		    //stop();
		    
		    
		    double diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
		    dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
		    
		    turnLeft(dTheta);
		    //turnRight(ROTATION_ERROR_CCW);
		    
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), false);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
			
			turnRight(90);
			
			left = false;
		    right = false;
		    leftDetection = 0;
		    rightDetection = 0;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()&&!left) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(left) {
			    		stop();
			    	}
			    }
		    } while(!left||!right);
		    //stop();
		    
		    
		    diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
		    dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
		    
		    turnLeft(dTheta);
		    //turnRight(ROTATION_ERROR_CCW);
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), false);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
			turnLeft(87);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    Odometer.setX(TILE_SIZE);
		    Odometer.setY(TILE_SIZE);
		    Odometer.setT(0);
		    position = new int[] {1,1,0};
		    
		} else if (START_CORNER == 1) {
			//Down right
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			
			double[] pos1 = new double[] {0,0,0};
			boolean isDetected = false;
			float d1 = ULTRASONIC.getData();
			float d2 = 0;
			
			// Rotate clockwise until you detect a falling edge
		    while(!isDetected) {
		    	try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.forward();
		        RIGHT_MOTOR.backward();
		        d2 = d1;
		        d1 = ULTRASONIC.getData();
		        if((d2 >= (STANDARD_DISTANCE_FALLING + NOTICE_MARGIN))&&(d1<= (STANDARD_DISTANCE_FALLING - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos1 = Odometer.getPosition();
		        }
		    }
		    Sound.beep();
		    
		    double[] pos2 = new double[] {0,0,0};
		    isDetected = false;
		    
		    d1 = ULTRASONIC.getData();
			d2 = 0;
			
		    // Rotate counter-clockwise until you detect another falling edge
		    while(!isDetected) {
		        try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.backward();
		        RIGHT_MOTOR.forward();
		        d2 = d1;
		        d1 = ULTRASONIC.getData();
		        if((d2 >= (STANDARD_DISTANCE_FALLING + NOTICE_MARGIN))&&(d1<= (STANDARD_DISTANCE_FALLING - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos2 = Odometer.getPosition();
		        }
		    }
		    
		    Sound.beep();
		    stop();
		    
		    // Calculate angle of local maximum based on two detected edges and use it to find 0° 
		    double dTheta = (-225 + (pos1[2]+pos2[2])/2 + 360)%360;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
		    //Turn to 0
		    turnTo(dTheta);
		    turnRight(ROTATION_ERROR_CW);
		    Odometer.resetTheta();//Reset theta
		    
		    
		    boolean left = false;
		    boolean right = false;
		    double leftDetection = 0;
		    double rightDetection = 0;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()&&!left) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(left) {
			    		stop();
			    	}
			    }
		    } while(!left||!right);
		    //stop();
		    
		    
		    double diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
		    dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
		    
		    turnLeft(dTheta);
		    //turnRight(ROTATION_ERROR_CCW);
		    
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), false);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
			
			turnLeft(90);
			
			left = false;
		    right = false;
		    leftDetection = 0;
		    rightDetection = 0;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()&&!left) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(left) {
			    		stop();
			    	}
			    }
		    } while(!left||!right);
		    //stop();
		    
		    
		    diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
		    dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
		    
		    turnLeft(dTheta);
		    //turnRight(ROTATION_ERROR_CCW);
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), false);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
			turnRight(93);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    Odometer.setX(7*TILE_SIZE);
		    Odometer.setY(TILE_SIZE);
		    Odometer.setT(0);
		    position = new int[] {7,1,0};
		    
		} else if (START_CORNER == 2) {
			//Up left
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			
			double[] pos1 = new double[] {0,0,0};
			boolean isDetected = false;
			float d1 = ULTRASONIC.getData();
			float d2 = 0;
			
			// Rotate clockwise until you detect a falling edge
		    while(!isDetected) {
		    	try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.forward();
		        RIGHT_MOTOR.backward();
		        d2 = d1;
		        d1 = ULTRASONIC.getData();
		        if((d2 >= (STANDARD_DISTANCE_FALLING + NOTICE_MARGIN))&&(d1<= (STANDARD_DISTANCE_FALLING - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos1 = Odometer.getPosition();
		        }
		    }
		    Sound.beep();
		    
		    double[] pos2 = new double[] {0,0,0};
		    isDetected = false;
		    
		    d1 = ULTRASONIC.getData();
			d2 = 0;
			
		    // Rotate counter-clockwise until you detect another falling edge
		    while(!isDetected) {
		        try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.backward();
		        RIGHT_MOTOR.forward();
		        d2 = d1;
		        d1 = ULTRASONIC.getData();
		        if((d2 >= (STANDARD_DISTANCE_FALLING + NOTICE_MARGIN))&&(d1<= (STANDARD_DISTANCE_FALLING - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos2 = Odometer.getPosition();
		        }
		    }
		    
		    Sound.beep();
		    stop();
		    
		    // Calculate angle of local maximum based on two detected edges and use it to find 0° 
		    double dTheta = (-225 - 90 + (pos1[2]+pos2[2])/2 + 360)%360;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
		    //Turn to 0
		    turnTo(dTheta);
		    turnRight(ROTATION_ERROR_CW);
		    Odometer.resetTheta();//Reset theta
		    
		    
		    boolean left = false;
		    boolean right = false;
		    double leftDetection = 0;
		    double rightDetection = 0;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()&&!left) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(left) {
			    		stop();
			    	}
			    }
		    } while(!left||!right);
		    //stop();
		    
		    
		    double diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
		    dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
		    
		    turnLeft(dTheta);
		    //turnRight(ROTATION_ERROR_CCW);
		    
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), false);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
			
			turnRight(90);
			
			left = false;
		    right = false;
		    leftDetection = 0;
		    rightDetection = 0;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()&&!left) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(left) {
			    		stop();
			    	}
			    }
		    } while(!left||!right);
		    //stop();
		    
		    
		    diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
		    dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
		    
		    turnLeft(dTheta);
		    //turnRight(ROTATION_ERROR_CCW);
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), false);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
			turnLeft(87);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    Odometer.setX(7*TILE_SIZE);
		    Odometer.setY(7*TILE_SIZE);
		    Odometer.setT(180);
		    position = new int[] {7,7,180};
		    
		} else if (START_CORNER == 3) {
			//Up right
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			
			double[] pos1 = new double[] {0,0,0};
			boolean isDetected = false;
			float d1 = ULTRASONIC.getData();
			float d2 = 0;
			
			// Rotate clockwise until you detect a falling edge
		    while(!isDetected) {
		    	try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.forward();
		        RIGHT_MOTOR.backward();
		        d2 = d1;
		        d1 = ULTRASONIC.getData();
		        if((d2 >= (STANDARD_DISTANCE_FALLING + NOTICE_MARGIN))&&(d1<= (STANDARD_DISTANCE_FALLING - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos1 = Odometer.getPosition();
		        }
		    }
		    Sound.beep();
		    
		    double[] pos2 = new double[] {0,0,0};
		    isDetected = false;
		    
		    d1 = ULTRASONIC.getData();
			d2 = 0;
			
		    // Rotate counter-clockwise until you detect another falling edge
		    while(!isDetected) {
		        try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.backward();
		        RIGHT_MOTOR.forward();
		        d2 = d1;
		        d1 = ULTRASONIC.getData();
		        if((d2 >= (STANDARD_DISTANCE_FALLING + NOTICE_MARGIN))&&(d1<= (STANDARD_DISTANCE_FALLING - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos2 = Odometer.getPosition();
		        }
		    }
		    
		    Sound.beep();
		    stop();
		    
		    // Calculate angle of local maximum based on two detected edges and use it to find 0° 
		    double dTheta = (-225 + (pos1[2]+pos2[2])/2 + 360)%360;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
		    //Turn to 0
		    turnTo(dTheta);
		    turnRight(ROTATION_ERROR_CW + 90);
		    Odometer.resetTheta();//Reset theta
		    
		    
		    boolean left = false;
		    boolean right = false;
		    double leftDetection = 0;
		    double rightDetection = 0;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()&&!left) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(left) {
			    		stop();
			    	}
			    }
		    } while(!left||!right);
		    //stop();
		    
		    
		    double diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
		    dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
		    
		    turnLeft(dTheta);
		    //turnRight(ROTATION_ERROR_CCW);
		    
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), false);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
			
			turnLeft(90);
			
			left = false;
		    right = false;
		    leftDetection = 0;
		    rightDetection = 0;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()&&!left) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	Sound.beep();
			    	if(left) {
			    		stop();
			    	}
			    }
		    } while(!left||!right);
		    //stop();
		    
		    
		    diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
		    dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
		    
		    turnLeft(dTheta);
		    //turnRight(ROTATION_ERROR_CCW);
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), false);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
			turnRight(93);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    Odometer.setX(TILE_SIZE);
		    Odometer.setY(7*TILE_SIZE);
		    Odometer.setT(0);
		    position = new int[] {1,7,0};
		    		    
		} else {
			Sound.beep();
			Sound.beep();
			Sound.beep();
			System.exit(0);
		}
	}

	private void stop() {
		LEFT_MOTOR.stop(true);
		RIGHT_MOTOR.stop(false);
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	private void moveToStart() {
		LEFT_MOTOR.setSpeed(NAVIGATING_SPEED);
	    RIGHT_MOTOR.setSpeed(NAVIGATING_SPEED);
	    LEFT_MOTOR.setAcceleration(ACCELERATION);
		RIGHT_MOTOR.setAcceleration(ACCELERATION);
		if(START_CORNER==0) {
			if(LL[0]==0&&LL[1]==0) {return;}
			double x = LL[0]*=TILE_SIZE;
			double y = LL[1]*=TILE_SIZE;
			double dX = x - TILE_SIZE;
			double dY = y - TILE_SIZE;
		
			turnTo(calculateTheta(dX, dY));
			
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
			
			LEFT_MOTOR.rotate(convertDistance(distance), true);
		    RIGHT_MOTOR.rotate(convertDistance(distance), false);
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    
		} else if(START_CORNER==1) {
			if(LL[0]==7&&LL[1]==0) {return;}
			double x = LL[0]*=TILE_SIZE;
			double y = LL[1]*=TILE_SIZE;
			double dX = x - 7*TILE_SIZE;
			double dY = y - TILE_SIZE;
			
			turnTo(calculateTheta(dX, dY));
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
			
			LEFT_MOTOR.rotate(convertDistance(distance), true);
		    RIGHT_MOTOR.rotate(convertDistance(distance), false);
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		} else if(START_CORNER==2) {
			if(LL[0]==7&&LL[1]==7) {return;}
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			double distance = (7 - LL[1])*TILE_SIZE;
			LEFT_MOTOR.rotate(convertDistance(distance), true);
		    RIGHT_MOTOR.rotate(convertDistance(distance), false);
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    turnTo(265);
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    distance = (7 - LL[0])*TILE_SIZE;
		    LEFT_MOTOR.setSpeed(NAVIGATING_SPEED);
		    RIGHT_MOTOR.setSpeed(NAVIGATING_SPEED);
		    LEFT_MOTOR.setAcceleration(ACCELERATION);
			RIGHT_MOTOR.setAcceleration(ACCELERATION);
			LEFT_MOTOR.rotate(convertDistance(distance), true);
		    RIGHT_MOTOR.rotate(convertDistance(distance), false);
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    turnTo(0);
		} else if(START_CORNER==3) {
			if(LL[0]==0&&LL[1]==7) {return;}
			double x = LL[0]*=TILE_SIZE;
			double y = LL[1]*=TILE_SIZE;
			double dX = x - TILE_SIZE;
			double dY = y - 7*TILE_SIZE;
			
			turnTo(calculateTheta(dX, dY));
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
			
			LEFT_MOTOR.rotate(convertDistance(distance), true);
		    RIGHT_MOTOR.rotate(convertDistance(distance), false);
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		}
		turnTo(0);
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		position[0] = LL[0];
		position[1] = LL[1];
		reLocalize();
	}
	
	private void reLocalize() {
		boolean left = false;
	    boolean right = false;
	    double leftDetection = 0;
	    double rightDetection = 0;
	    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
	    LEFT_MOTOR.forward();
	    RIGHT_MOTOR.forward();
	    do {
		    if(detectLineLeft()&&!left) {
		    	left = true;
		    	leftDetection = LEFT_MOTOR.getTachoCount();
		    	Sound.beep();
		    	if(right) {
		    		stop();
		    	}
		    }
		    if(detectLineRight()&&!right) {
		    	right = true;
		    	rightDetection = LEFT_MOTOR.getTachoCount();
		    	Sound.beep();
		    	if(left) {
		    		stop();
		    	}
		    }
	    } while(!left||!right);
	    //stop();
	    
	    
	    double diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
	    double dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
	    
	    turnLeft(dTheta);
	    //turnRight(ROTATION_ERROR_CCW);
	    
	    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), true);
		RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), false);
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		
		turnRight(90);
		
		left = false;
	    right = false;
	    leftDetection = 0;
	    rightDetection = 0;
	    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
	    LEFT_MOTOR.forward();
	    RIGHT_MOTOR.forward();
	    do {
		    if(detectLineLeft()&&!left) {
		    	left = true;
		    	leftDetection = LEFT_MOTOR.getTachoCount();
		    	Sound.beep();
		    	if(right) {
		    		stop();
		    	}
		    }
		    if(detectLineRight()&&!right) {
		    	right = true;
		    	rightDetection = LEFT_MOTOR.getTachoCount();
		    	Sound.beep();
		    	if(left) {
		    		stop();
		    	}
		    }
	    } while(!left||!right);
	    //stop();
	    
	    
	    diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
	    dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
	    
	    turnLeft(dTheta);
	    //turnRight(ROTATION_ERROR_CCW);
	    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
	    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), true);
		RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_WHEEL_DIFF), false);
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
	    
		turnLeft(87);
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
	}
	
	private void travelTo(int x, int y) {
		LEFT_MOTOR.setSpeed(SEARCHING_SPEED);
    	RIGHT_MOTOR.setSpeed(SEARCHING_SPEED);
    	LEFT_MOTOR.setAcceleration(500);
    	RIGHT_MOTOR.setAcceleration(500);
    	
		double[] currPos = Odometer.getPosition();
		x*=TILE_SIZE;
		y*=TILE_SIZE;
		double dX = x-currPos[0];
		double dY = y-currPos[1];
		if(dY>0) {
			turnTo(0);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			LEFT_MOTOR.rotate(convertDistance(dY), true);
		    RIGHT_MOTOR.rotate(convertDistance(dY), true);
		    do{
		    	double distance = ULTRASONIC.getData();
		    	if(distance<=5) {
		    		stop();
		    		currPos = Odometer.getPosition();
		    		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    		LEFT_MOTOR.setSpeed(APPROACHING_SPEED);
		        	RIGHT_MOTOR.setSpeed(APPROACHING_SPEED);
		        	LEFT_MOTOR.setAcceleration(500);
		        	RIGHT_MOTOR.setAcceleration(500);
		        	LEFT_MOTOR.rotate(convertDistance(5), true);
		    	    RIGHT_MOTOR.rotate(convertDistance(5), false);
		    	    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    	    //Detecting
		    	    Sound.twoBeeps();
		    	    LEFT_MOTOR.rotate(-convertDistance(5), true);
		    	    RIGHT_MOTOR.rotate(-convertDistance(5), false);
		    	    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    	    //Hitting
		    	    LEFT_MOTOR.rotate(convertDistance((y-currPos[1])), true);
		    	    RIGHT_MOTOR.rotate(convertDistance((y-currPos[1])), true);
		    	}
		    }while(LEFT_MOTOR.isMoving()&&RIGHT_MOTOR.isMoving());
		} else if(dY<0) {
			turnTo(180);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			LEFT_MOTOR.rotate(convertDistance(-dY), true);
		    RIGHT_MOTOR.rotate(convertDistance(-dY), true);
		    do{
		    	double distance = ULTRASONIC.getData();
		    	if(distance<=5) {
		    		stop();
		    		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    		LEFT_MOTOR.setSpeed(APPROACHING_SPEED);
		        	RIGHT_MOTOR.setSpeed(APPROACHING_SPEED);
		        	LEFT_MOTOR.setAcceleration(500);
		        	RIGHT_MOTOR.setAcceleration(500);
		        	LEFT_MOTOR.rotate(convertDistance(5), true);
		    	    RIGHT_MOTOR.rotate(convertDistance(5), false);
		    	    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    	    //Detecting
		    	    Sound.twoBeeps();
		    	    LEFT_MOTOR.rotate(-convertDistance(5), true);
		    	    RIGHT_MOTOR.rotate(-convertDistance(5), false);
		    	    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    	    //Hitting
		    	    currPos = Odometer.getPosition();
		    	    LEFT_MOTOR.rotate(convertDistance((currPos[1]-y)), true);
		    	    RIGHT_MOTOR.rotate(convertDistance((currPos[1]-y)), true);
		    	}
		    } while(LEFT_MOTOR.isMoving()&&RIGHT_MOTOR.isMoving());
		    
		}
		currPos = Odometer.getPosition();
		if(dX>0) {
			turnTo(90);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			LEFT_MOTOR.rotate(convertDistance(dX), true);
		    RIGHT_MOTOR.rotate(convertDistance(dX), false);
		    do{
		    	double distance = ULTRASONIC.getData();
		    	if(distance<=5) {
		    		stop();
		    		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    		LEFT_MOTOR.setSpeed(APPROACHING_SPEED);
		        	RIGHT_MOTOR.setSpeed(APPROACHING_SPEED);
		        	LEFT_MOTOR.setAcceleration(500);
		        	RIGHT_MOTOR.setAcceleration(500);
		        	LEFT_MOTOR.rotate(convertDistance(5), true);
		    	    RIGHT_MOTOR.rotate(convertDistance(5), false);
		    	    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    	    //Detecting
		    	    Sound.twoBeeps();
		    	    LEFT_MOTOR.rotate(-convertDistance(5), true);
		    	    RIGHT_MOTOR.rotate(-convertDistance(5), false);
		    	    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    	    //Hitting
		    	    currPos = Odometer.getPosition();
		    	    LEFT_MOTOR.rotate(convertDistance((x-currPos[0])), true);
		    	    RIGHT_MOTOR.rotate(convertDistance((x-currPos[0])), true);
		    	}
		    } while(LEFT_MOTOR.isMoving()&&RIGHT_MOTOR.isMoving());
		} else if(dX<0) {
			turnTo(270);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			LEFT_MOTOR.rotate(convertDistance(-dX), true);
		    RIGHT_MOTOR.rotate(convertDistance(-dX), true);
		    do{
		    	double distance = ULTRASONIC.getData();
		    	if(distance<=5) {
		    		stop();
		    		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    		LEFT_MOTOR.setSpeed(APPROACHING_SPEED);
		        	RIGHT_MOTOR.setSpeed(APPROACHING_SPEED);
		        	LEFT_MOTOR.setAcceleration(500);
		        	RIGHT_MOTOR.setAcceleration(500);
		        	LEFT_MOTOR.rotate(convertDistance(5), true);
		    	    RIGHT_MOTOR.rotate(convertDistance(5), false);
		    	    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    	    //Detecting
		    	    Sound.twoBeeps();
		    	    LEFT_MOTOR.rotate(-convertDistance(5), true);
		    	    RIGHT_MOTOR.rotate(-convertDistance(5), false);
		    	    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		    	    //Hitting
		    	    currPos = Odometer.getPosition();
		    	    LEFT_MOTOR.rotate(convertDistance((currPos[0]-x)), true);
		    	    RIGHT_MOTOR.rotate(convertDistance((currPos[0]-x)), true);
		    	}
		    } while(LEFT_MOTOR.isMoving()&&RIGHT_MOTOR.isMoving());
		}
	}
	
	private int[][] calculateNavigationPoint(){
		int size = UR[0] - LL[0] + 1;
		int[][] coordinates = new int[2][size*2];
		for(int i = 0; i<size; i++) {
			if(i%2==0) {
				//START X -> FINISH X
				coordinates[0][i*2] = LL[0];
				coordinates[0][i*2+1] = UR[0];
				coordinates[1][i*2] = UR[1] + i;
				coordinates[1][i*2+1] = UR[1] + i;
			} else {
				//FINISH X -> START X
				coordinates[0][i*2] = UR[0];
				coordinates[0][i*2+1] = LL[0];
				coordinates[1][i*2] = UR[1] + i;
				coordinates[1][i*2+1] = UR[1] + i;
			}
		}
		return coordinates;
	}
	
	private void search() {
		int[][] coordinates = calculateNavigationPoint();
		int size = coordinates[0].length;
		for(int i = 0; i<size; i++) {
			travelTo(coordinates[0][i], coordinates[1][i]);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		}
		travelTo(UR[0],UR[1]);
	}
	
	private double calculateTheta(double dX, double dY) {
		double headingTheta = 0;
		if(dX>0) {
		  if(dY>0) {
			headingTheta = Math.toDegrees(Math.atan(dX/dY));
		  } else if(dY<0) {
			headingTheta = (Math.toDegrees(Math.atan(dX/dY))+180)%180;
		  } else {
			headingTheta = 90;
		  }
		} else if (dX<0){
		  if(dY>0) {
			headingTheta = (Math.toDegrees(Math.atan(dX/dY))+360)%360;
		  } else if(dY<0) {
			headingTheta = Math.toDegrees(Math.atan(dX/dY))+180;
		  } else {
			headingTheta = 270;
		  }
		} else {
		  if(dY>0) {
			headingTheta = 0;
		  } else if(dX<0) {
			headingTheta = 180;
		  } else {
			headingTheta = 0;
		  }
		}
		return headingTheta;
	  }
	
	private void turnTo(double Theta) {// Theta is in the range of [0, 359.9]
		stop();
		double[] pos = Odometer.getPosition();
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {};
		
		if(pos[2]>Theta) {
			double dTheta = pos[2] - Theta;
			if(dTheta>180) {
				turnRight(360-dTheta);
			} else {
				turnLeft(dTheta);
			}
		} else if(pos[2]<Theta) {
			double dTheta = Theta - pos[2];
			if(dTheta>180) {
				turnLeft(360-dTheta);
			} else {
				turnRight(dTheta);
			}
		}
	}
	
	private void turnLeft(double dTheta) {
		//if(dTheta<1) {return;}
		stop();
		LEFT_MOTOR.setSpeed(TURNING_SPEED);
		RIGHT_MOTOR.setSpeed(TURNING_SPEED);
		LEFT_MOTOR.setAcceleration(ACCELERATION);
		RIGHT_MOTOR.setAcceleration(ACCELERATION);
		
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		
		if (dTheta<0) {
			turnRight(-dTheta);
		} else {
		    LEFT_MOTOR.rotate(-convertAngle(dTheta), true);
		    RIGHT_MOTOR.rotate(convertAngle(dTheta), false);
		}
	}
	  
	private void turnRight(double dTheta) {
		//if(dTheta<1) {return;}
		stop();
		LEFT_MOTOR.setSpeed(TURNING_SPEED);
		RIGHT_MOTOR.setSpeed(TURNING_SPEED);
		LEFT_MOTOR.setAcceleration(ACCELERATION);
		RIGHT_MOTOR.setAcceleration(ACCELERATION);
		
		try {Thread.sleep(100);} catch (InterruptedException e) {}
		
		if (dTheta<0) {
		    turnLeft(-dTheta);
		} else {
		    LEFT_MOTOR.rotate(convertAngle(dTheta), true);
		    RIGHT_MOTOR.rotate(-convertAngle(dTheta), false);
		}
	}
	  
	private int convertAngle(double angle) {return convertDistance(Math.PI * TRACK * angle / 360.0);}
	  
	private int convertDistance(double distance) {return (int) ((180.0 * distance) / (Math.PI * RADIUS));}
	
	private boolean detectLineLeft() {
		float leftData = LOCALIZING_LEFT.getData(MEASURING_SCOPE);
		return 1.15 < leftData/BLACK_LINE_LEFT || 0.85 > leftData/BLACK_LINE_LEFT;
	}
	
	private boolean detectLineRight() {
		float rightData = LOCALIZING_RIGHT.getData(MEASURING_SCOPE);
		return 1.15 < rightData/BLACK_LINE_RIGHT || 0.85 > rightData/BLACK_LINE_RIGHT;
	}
	
	/**
	 * Unit test function
	 */
	public void demo() {
		//localize();
		//moveToStart();
		//search();
		
	}
}
