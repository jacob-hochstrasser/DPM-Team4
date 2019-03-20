package ca.mcgill.ecse211.Project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/** 
 * This is a class that controls the movements of the Lego EV3 robot.
 * 
 * @author Pengnan Fan
 *
 */

public class Navigation {
	
	//-----<Important Constant>-----//
	/**
	 * This parameter stands for the radius of wheels.
	 */
	public static final double RADIUS = 1.6;//cm. It is certifed by testing from Pengnan.
	/**
	 * This parameter stands for the distances between two tracks.
	 */
	public static final double TRACK = 24.5;//cm. It is certified by testing from Pengnan.
	/**
	 * Angular acceleration of the motors
	 */
	volatile private static int ACCELERATION = 1200;
	/**
	 * Angular velocity of the motors
	 */
	volatile private static int SPEED = 350;
	/**
	 * Time interval used for thread timing
	 */
	private static final int TIME_INTERVAL = 15;//ms
	private static final int MEASURING_INTERVAL = 0;
	/**
	 * Size of floor tiles
	 */
	private static final double TILE_SIZE = 30.28;//cm
	/**
	 * Constant used to trigger falling edge detection
	 */
	private static final double EDGE_DISTANCE = 1.5*TILE_SIZE;
	/**
	 * Margin used for noise handling during falling edge localization
	 */
	private static final double NOTICE_MARGIN = 1;//cm
	/**
	 * Track distance of localizing light sensors
	 */
	private static final double LS_TK_DIS = 8.75;//cm
	/**
	 * Distance between the two light sensors
	 */
	private static final double LS_DIFF = 15;
	/**
	 * Number of color samples taken during light sensor initialization
	 */
	private static final int INITIALIZING_SCOPE = 25;
	/**
	 * Number of color samples taken during light localization
	 */
	private static final int MEASURING_SCOPE = 1;
	/**
	 * Numbers of degrees error after calculations to correct by during localization.
	 */
	private static final double ROTATION_ERROR = 10;
	
	private static final float CAN_DETECTION = 30;
	
	/**
	 * Color intensity baseline for left localizing light sensor
	 */
	private float LINE_L;
	/**
	 * Color intensity baseline for right localizing light sensor
	 */
	private float LINE_R;
	
	//-----<Searching Coordinates and Starting Corner>-----//
	/**
	 * The corner the robot starts in {0 -> bottom left, 1 -> bottom right, 2 -> top right, 3 -> top left}
	 */
	private int START_CORNER = 0;// 0 -> down left, 1 -> down right, 2 -> up right, 3 -> up left
	/**
	 * The lower left corner of the table
	 */
	private int[] LL;
	/**
	 * The upper right corner of the table
	 */
	private int[] UR;
		
	//-----<Motor>-----//
	/**
	 * This object stands for the left motor.
	 */
	public static EV3LargeRegulatedMotor LEFT_MOTOR;
	/**
	 * This object stands for the right motor.
	 */
	public static EV3LargeRegulatedMotor RIGHT_MOTOR;
	
	//-----<Sensors>-----//
	/**
	 * Sensor poller for the left localizing light sensor; acts like a wrapper for the sensor itself
	 */
	private static SensorPoller LS_L = new LightSensorPoller("S2", MEASURING_INTERVAL, false);
	/**
	 * Sensor poller for the right localizing light sensor; acts like a wrapper for the sensor itself
	 */
	private static SensorPoller LS_R = new LightSensorPoller("S1", MEASURING_INTERVAL, false);
	/**
	 * Sensor poller for the ultrasonic sensor; acts like a wrapper for the sensor itself
	 */
	private static SensorPoller US = new UltrasonicSensorPoller("S3", MEASURING_INTERVAL);
	
	//-----<Position>-----//
	/**
	 * 
	 */
	private double[] position = new double[] {0,0,0};
	
	//-----<Nested MotorController>-----//
	private static Navigation NV;
	
	private static Identifier ID = BetaDemoApplication.ID;
	
	private boolean foundCan = false;
	
	protected static final boolean DEBUG = true;
	
	/**
	 * This is only for block testing
	 */
	void demo() {
		LEFT_MOTOR.setSpeed(200);
		RIGHT_MOTOR.setSpeed(200);
		LEFT_MOTOR.backward();
		RIGHT_MOTOR.forward();
	}
	
	/**
	 * 
	 * This method is used to get a Navigation object. 
	 * Note: Only one of this type of object can be accessed.
	 *
	 * @param leftMotor
	 * This input indicates the string of the port used for left motor.
	 * 
	 * @param rightMotor
	 * This input indicates the string of the port used for right motor.
	 * 
	 * @return 
	 * This method returns a Navigation object nested in the class.
	 * 
	 */
	public static Navigation getNavigation(String leftMotor, String rightMotor) {
		if(NV==null) {
			NV = new Navigation(leftMotor, rightMotor);
		}
		return NV;
	}
	
	/**
	 * This method sets up the searching range and the starting corner.
	 * 
	 * @param LL
	 * This input is a 2-element array indicating the lower left coordinates of the searching range
	 * 
	 * @param UR
	 * This input is a 2-element array indicating the upper right coordinates of the searching range
	 * 
	 * @param START_CORNER
	 * This input suggests the starting corner
	 * 
	 */
	public void setup(int[] LL, int[] UR, int START_CORNER) {
		this.LL = LL;
		this.UR = UR;
		this.START_CORNER = START_CORNER;
		initializeAcceleration();
		initializeSpeed();
		initializeLightSensors();
	}
	
	private void initializeLightSensors() {
		float sum_left = 0;
		float sum_right = 0;
		for(int i = 0; i<INITIALIZING_SCOPE; i++) {
			sum_left+=LS_L.getData();
			sum_right+=LS_R.getData();
		}
		LINE_L = sum_left/INITIALIZING_SCOPE;
		LINE_R = sum_right/INITIALIZING_SCOPE;
	}
	
	private Navigation(String leftMotor, String rightMotor) {
		LEFT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort(leftMotor));
		RIGHT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort(rightMotor));
		initializeAcceleration();
		initializeSpeed();
		initializeLightSensors();
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
	
	/**
	 * This method drives the robot to a given coordinates in the unit of a tile size.
	 * 
	 * @param x
	 * This input indicates the x coordinate in the unit of a tile size.
	 * @param y
	 * This input indicates the y coordinate in the unit of a tile size
	 */
	public void travelTo(double x, double y) {
	    position = Odometer.getPosition();
		x*=TILE_SIZE;
		y*=TILE_SIZE;
		double dX = x - position[0];
		double dY = y - position[1];
		
		turnTo(calculateTheta(dX, dY));
		
		double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
		
		moveForward(distance);

	    //Sound.beep();
	}
	
	/**
	 * This method drives the robot to a given coordinates in the unit of a tile size.
	 * @param coords
	 * This input is a 2-element array for the coodinates in the unit of a tile size.
	 */
	public void travelTo(int[] coords) {
	    travelTo(coords[0], coords[1]);
        //Sound.beep();
    }
	
	/**
	 * This method drives the robot alone with the rows. It will search for any can it finds.
	 */
	public void search() {
		int n_col = UR[0] - LL[0];
		int n_row = UR[1] - LL[1];
		for(int row = 0; row<n_row; row++) {
			if(row%2==0) {
				// going from LL[0] to UR[0]
				for(int col = 0; col<n_col; col++) {
					position = Odometer.getPosition();
					float can_dis = US.getData(); 
					while(!isReached(col+LL[0], row+LL[1])) {
						turnTo(calculateTheta((row+LL[1])*TILE_SIZE - position[0], (col+LL[0])*TILE_SIZE - position[1]));
						LEFT_MOTOR.forward();
						RIGHT_MOTOR.forward();
						if (can_dis < CAN_DETECTION) {
							stop();
							turnLeft(180);
							moveBackward(CAN_DETECTION);
							//TODO Start measuring color
							foundCan = ID.isTargetCan();
							moveForward(CAN_DETECTION);
							turnLeft(180);
						}
						
					}
					stop();
				}
				// move one row above
				position = Odometer.getPosition();
				float can_dis = US.getData();
				while(!isReached(UR[0], row+LL[1]+1)) {
					turnTo(calculateTheta(UR[0]*TILE_SIZE - position[0], (row+LL[1])*TILE_SIZE - position[1]));
					LEFT_MOTOR.forward();
					RIGHT_MOTOR.forward();
					if (can_dis < CAN_DETECTION) {
						stop();
						turnLeft(180);
						moveBackward(CAN_DETECTION);
						//TODO Start measuring color
						foundCan = ID.isTargetCan();
						moveForward(CAN_DETECTION);
						turnLeft(180);
					}
				}
			} else {
				// going from UR[0] to LL[0]
				for(int col = n_col; col>=0; col--) {
					position = Odometer.getPosition();
					float can_dis = US.getData();
					while(!isReached(col+LL[0], row+LL[1])) {
						turnTo(calculateTheta((row+LL[1])*TILE_SIZE - position[0], (col+LL[0])*TILE_SIZE - position[1]));
						LEFT_MOTOR.forward();
						RIGHT_MOTOR.forward();
						if (can_dis < CAN_DETECTION) {
							stop();
							turnLeft(180);
							moveBackward(CAN_DETECTION);
							//TODO Start measuring color
							foundCan = ID.isTargetCan();
							moveForward(CAN_DETECTION);
							turnLeft(180);
						}
						
					}
					stop();
				}
				position = Odometer.getPosition();
				float can_dis = US.getData();
				while(!isReached(UR[0], row+LL[1]+1)) {
					turnTo(calculateTheta(LL[0]*TILE_SIZE - position[0], (row+LL[1])*TILE_SIZE - position[1]));
					LEFT_MOTOR.forward();
					RIGHT_MOTOR.forward();
					if (can_dis < CAN_DETECTION) {
						stop();
						turnLeft(180);
						moveBackward(CAN_DETECTION);
						//TODO Start measuring color
						foundCan = ID.isTargetCan();
						moveForward(CAN_DETECTION);
						turnLeft(180);
					}
				}
				stop();
			}
			
		}
	}
	
	private boolean isReached(double x, double y) {
		position = Odometer.getPosition();
		return position[0]==(x*TILE_SIZE)&&position[1]==(y*TILE_SIZE);
	}
	
	/**
	 * This method resets the acceleration and re-initializes the motor with the new acceleration.
	 * 
	 * @param newAcceleration
	 * This input is an int for the new acceleration.
	 * 
	 */
	public void setAcceleration(int newAcceleration) {
		ACCELERATION = newAcceleration;
		initializeAcceleration();
	}
	
	/**
	 * This method returns the current speed value being used;
	 * 
	 * @return SPEED:int
	 * The current speed of the motors (deg/s).
	 */
	public int getSpeed() {
	    return Navigation.SPEED;
	}
	
	/**
	 * 
	 * This method resets the speed and re-initializes the motor with the new speed.
	 * 
	 * @param newSpeed:int
	 * This input is an int for the new speed.
	 * 
	 */
	public void setSpeed(int newSpeed) {
		SPEED = newSpeed;
		initializeSpeed();
	}
	
	/**
	 * 
	 * This method initializes the motors with the current speed.
	 * 
	 */
	public void initializeSpeed() {
		LEFT_MOTOR.setSpeed(SPEED);
		RIGHT_MOTOR.setSpeed(SPEED);
	}

	/**
	 * 
	 * This method initializes the motors with the current acceleration.
	 * 
	 */
	public void initializeAcceleration() {
		LEFT_MOTOR.setAcceleration(ACCELERATION);
		RIGHT_MOTOR.setAcceleration(ACCELERATION);
	}
	
	private boolean detectLineLeft() {
		float leftData = LS_L.getData(MEASURING_SCOPE);
		return 1.15 < leftData/LINE_L || 0.85 > leftData/LINE_L;
	}
	
	private boolean detectLineRight() {
		float rightData = LS_R.getData(MEASURING_SCOPE);
		return 1.15 < rightData/LINE_R || 0.85 > rightData/LINE_R;
	}

	/**
	 * 
	 * This method re-localizes the robot with light sensors during the navigation. 
	 * 
	 */
	public void reLocalize() {
		boolean left = false;
	    boolean right = false;
	    double leftDetection = 0;
	    double rightDetection = 0;
	    int speed = Navigation.SPEED;
	    setSpeed(Math.max((int)(0.25 * SPEED), 200));
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
	    setSpeed(speed);
	    
	    double diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
	    double dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
	    
	    turnLeft(dTheta);
	    //turnRight(ROTATION_ERROR_CCW);
	    
	    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), true);
		RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), false);
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		
		turnRight(90);
		
		left = false;
	    right = false;
	    leftDetection = 0;
	    rightDetection = 0;
	    setSpeed(Math.max((int)(0.25 * SPEED), 200));
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
	    
	    setSpeed(speed);
	    diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
	    dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
	    
	    turnLeft(dTheta);
	    //turnRight(ROTATION_ERROR_CCW);
	    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
	    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), true);
		RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), false);
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
	    
		turnLeft(90);
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
	}
	
	/**
	 * 
	 * This method localizes the robot at the starting corner.
	 * 
	 */
	
	public void localize() {
		
		if (START_CORNER == 0) {
			//Down left
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			
			double[] pos1 = new double[] {0,0,0};
			boolean isDetected = false;
			float d1 = US.getData();
			float d2 = 0;
			
			// Rotate clockwise until you detect a falling edge
		    while(!isDetected) {
		    	try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.forward();
		        RIGHT_MOTOR.backward();
		        d2 = d1;
		        d1 = US.getData();
		        if((d2 >= (EDGE_DISTANCE + NOTICE_MARGIN))&&(d1<= (EDGE_DISTANCE - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos1 = Odometer.getPosition();
		        }
		    }
		    //Sound.beep();
		    
		    double[] pos2 = new double[] {0,0,0};
		    isDetected = false;
		    
		    d1 = US.getData();
			d2 = 0;
			
			LEFT_MOTOR.backward();
            RIGHT_MOTOR.forward();
            try {Thread.sleep(100 * TIME_INTERVAL);} catch (InterruptedException e) {}
			
		    // Rotate counter-clockwise until you detect another falling edge
		    while(!isDetected) {
		        try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        //LEFT_MOTOR.backward();
		        //RIGHT_MOTOR.forward();
		        d2 = d1;
		        d1 = US.getData();
		        if((d2 >= (EDGE_DISTANCE + NOTICE_MARGIN))&&(d1<= (EDGE_DISTANCE - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos2 = Odometer.getPosition();
		        }
		    }
		    
		    //Sound.beep();
		    stop();
		    
		    // Calculate angle of local maximum based on two detected edges and use it to find 0° 
		    double dTheta = (-225 - 90 + (pos1[2]+pos2[2])/2 + 360)%360;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
		    //Turn to 0
		    
		    turnTo(dTheta);
		    turnRight(2 * ROTATION_ERROR);
		    //turnRight(4.5 * ROTATION_ERROR);
		    Odometer.resetTheta();//Reset theta
		    if(DEBUG) {
		        if(Button.waitForAnyPress() == Button.ID_ESCAPE) System.exit(0);
		    }
		    
		    boolean left = false;
		    boolean right = false;
		    double leftDetection = 0;
		    double rightDetection = 0;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    int speed = Navigation.SPEED;
		    setSpeed(Math.max((int)(0.25 * SPEED), 200));
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()&&!left) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    	//Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	//Sound.beep();
			    	if(left) {
			    		stop();
			    	}
			    }
		    } while(!left||!right);
		    //stop();
		    setSpeed(speed);
		    
		    double diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
		    dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
		    
		    turnLeft(dTheta);
		    //turnRight(ROTATION_ERROR_CCW);
		    
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), false);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
			
			turnRight(90);
			
			left = false;
		    right = false;
		    leftDetection = 0;
		    rightDetection = 0;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    setSpeed(Math.max((int)(0.25 * SPEED), 200));
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()&&!left) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    	//Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	//Sound.beep();
			    	if(left) {
			    		stop();
			    	}
			    }
		    } while(!left||!right);
		    //stop();
		    
		    setSpeed(speed);
		    diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
		    dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
		    
		    turnLeft(dTheta);
		    //turnRight(ROTATION_ERROR_CCW);
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), false);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
			turnLeft(90 + ROTATION_ERROR);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    Odometer.setX(TILE_SIZE);
		    Odometer.setY(TILE_SIZE);
		    Odometer.setT(0);
		    position = new double[] {1,1,0};
		    
		} else if (START_CORNER == 1) {
			//Down right
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			
			double[] pos1 = new double[] {0,0,0};
			boolean isDetected = false;
			float d1 = US.getData();
			float d2 = 0;
			
			// Rotate clockwise until you detect a falling edge
		    while(!isDetected) {
		    	try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.forward();
		        RIGHT_MOTOR.backward();
		        d2 = d1;
		        d1 = US.getData();
		        if((d2 >= (EDGE_DISTANCE + NOTICE_MARGIN))&&(d1<= (EDGE_DISTANCE - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos1 = Odometer.getPosition();
		        }
		    }
		    //Sound.beep();
		    
		    double[] pos2 = new double[] {0,0,0};
		    isDetected = false;
		    
		    d1 = US.getData();
			d2 = 0;
			
		    // Rotate counter-clockwise until you detect another falling edge
		    while(!isDetected) {
		        try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.backward();
		        RIGHT_MOTOR.forward();
		        d2 = d1;
		        d1 = US.getData();
		        if((d2 >= (EDGE_DISTANCE + NOTICE_MARGIN))&&(d1<= (EDGE_DISTANCE - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos2 = Odometer.getPosition();
		        }
		    }
		    
		    //Sound.beep();
		    stop();
		    
		    // Calculate angle of local maximum based on two detected edges and use it to find 0° 
		    double dTheta = (-225 + (pos1[2]+pos2[2])/2 + 360)%360;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
		    //Turn to 0
		    turnTo(dTheta);
		    turnRight(ROTATION_ERROR);
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
			    	//Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	//Sound.beep();
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
		    
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), false);
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
			    	//Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	//Sound.beep();
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
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), false);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
			turnRight(93);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    Odometer.setX(7*TILE_SIZE);
		    Odometer.setY(TILE_SIZE);
		    Odometer.setT(0);
		    position = new double[] {7,1,0};
		    
		} else if (START_CORNER == 2) {
			//Up left
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			
			double[] pos1 = new double[] {0,0,0};
			boolean isDetected = false;
			float d1 = US.getData();
			float d2 = 0;
			
			// Rotate clockwise until you detect a falling edge
		    while(!isDetected) {
		    	try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.forward();
		        RIGHT_MOTOR.backward();
		        d2 = d1;
		        d1 = US.getData();
		        if((d2 >= (EDGE_DISTANCE + NOTICE_MARGIN))&&(d1<= (EDGE_DISTANCE - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos1 = Odometer.getPosition();
		        }
		    }
		    //Sound.beep();
		    
		    double[] pos2 = new double[] {0,0,0};
		    isDetected = false;
		    
		    d1 = US.getData();
			d2 = 0;
			
		    // Rotate counter-clockwise until you detect another falling edge
		    while(!isDetected) {
		        try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.backward();
		        RIGHT_MOTOR.forward();
		        d2 = d1;
		        d1 = US.getData();
		        if((d2 >= (EDGE_DISTANCE + NOTICE_MARGIN))&&(d1<= (EDGE_DISTANCE - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos2 = Odometer.getPosition();
		        }
		    }
		    
		    //Sound.beep();
		    stop();
		    
		    // Calculate angle of local maximum based on two detected edges and use it to find 0° 
		    double dTheta = (-225 - 90 + (pos1[2]+pos2[2])/2 + 360)%360;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
		    //Turn to 0
		    turnTo(dTheta);
		    turnRight(ROTATION_ERROR);
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
			    	//Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	//Sound.beep();
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
		    
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), false);
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
			    	//Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	//Sound.beep();
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
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), false);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
			turnLeft(87);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    Odometer.setX(7*TILE_SIZE);
		    Odometer.setY(7*TILE_SIZE);
		    Odometer.setT(180);
		    position = new double[] {7,7,180};
		    
		} else if (START_CORNER == 3) {
			//Up right
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
			
			double[] pos1 = new double[] {0,0,0};
			boolean isDetected = false;
			float d1 = US.getData();
			float d2 = 0;
			
			// Rotate clockwise until you detect a falling edge
		    while(!isDetected) {
		    	try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.forward();
		        RIGHT_MOTOR.backward();
		        d2 = d1;
		        d1 = US.getData();
		        if((d2 >= (EDGE_DISTANCE + NOTICE_MARGIN))&&(d1<= (EDGE_DISTANCE - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos1 = Odometer.getPosition();
		        }
		    }
		    //Sound.beep();
		    
		    double[] pos2 = new double[] {0,0,0};
		    isDetected = false;
		    
		    d1 = US.getData();
			d2 = 0;
			
		    // Rotate counter-clockwise until you detect another falling edge
		    while(!isDetected) {
		        try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		        LEFT_MOTOR.backward();
		        RIGHT_MOTOR.forward();
		        d2 = d1;
		        d1 = US.getData();
		        if((d2 >= (EDGE_DISTANCE + NOTICE_MARGIN))&&(d1<= (EDGE_DISTANCE - NOTICE_MARGIN))) {
		        	stop();
		        	isDetected = true;
		        	pos2 = Odometer.getPosition();
		        }
		    }
		    
		    //Sound.beep();
		    stop();
		    
		    // Calculate angle of local maximum based on two detected edges and use it to find 0° 
		    double dTheta = (-225 + (pos1[2]+pos2[2])/2 + 360)%360;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
		    //Turn to 0
		    turnTo(dTheta);
		    turnRight(ROTATION_ERROR + 90);
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
			    	//Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	//Sound.beep();
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
		    
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), false);
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
			    	//Sound.beep();
			    	if(right) {
			    		stop();
			    	}
			    }
			    if(detectLineRight()&&!right) {
			    	right = true;
			    	rightDetection = LEFT_MOTOR.getTachoCount();
			    	//Sound.beep();
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
		    LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), true);
			RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), false);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
			turnRight(93);
			try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    Odometer.setX(TILE_SIZE);
		    Odometer.setY(7*TILE_SIZE);
		    Odometer.setT(0);
		    position = new double[] {1,7,0};
		    		    
		} else {
			Sound.beep();
			Sound.beep();
			Sound.beep();
			System.exit(0);
		}
		Sound.beep();
	}
	
	/**
	 * 
	 * This method drives the robot backward to a given distance. It will automatically call moveForward if the distance is negative.
	 * 
	 * @param distance
	 * This input shows how much distance the robot should move backward in centimeter.
	 * 
	 */
	public void moveBackward(double distance) {
		if(distance<0) {moveForward(distance);}
		LEFT_MOTOR.rotate(-convertDistance(distance), true);
		RIGHT_MOTOR.rotate(-convertDistance(distance), false);
			
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	/**
	 * This method drives the robot forward to a given distance. It will automatically call moveBackward if the distance is negative.
	 * 
	 * @param distance:int
	 * This input shows how much distance the robot should move forward in centimeter.
	 * 
	 */
	public void moveForward(double distance) {
		if(distance<0) {moveBackward(distance);}
		
		LEFT_MOTOR.rotate(convertDistance(distance), true);
		RIGHT_MOTOR.rotate(convertDistance(distance), false);
		
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	/**
	 * This method turns the robot to the given direction in degrees.
	 * 
	 * @param Theta
	 * This input indicates which direction should the robot turn to in degrees.
	 */
	
	public void turnTo(double Theta) {// Theta is in the range of [0, 359.9]
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
	
	/**
	 * 
	 * This method turns the robot clockwise to a given degree. It will automatically call turnRight if the input degree is negative.
	 * 
	 * @param dTheta
	 * This input shows how much the robot should turn clockwise in degrees.
	 * 
	 */
	public void turnLeft(double dTheta) {
		if (dTheta<0) {
			turnRight(-dTheta);
		} else {
			LEFT_MOTOR.rotate(-convertAngle(dTheta), true);
			RIGHT_MOTOR.rotate(convertAngle(dTheta), false);
		}
		
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	/**
	 * This method turns the robot counter clockwise to a given degree. It will automatically call turnLeft if the input degree is negative.
	 * 
	 * @param dTheta
	 * This input shows how much the robot should turn counter clockwise in degrees.
	 * 
	 */
	public void turnRight(double dTheta) {
		if (dTheta<0) {
		    turnLeft(-dTheta);
		} else {
			LEFT_MOTOR.rotate(convertAngle(dTheta), true);
			RIGHT_MOTOR.rotate(-convertAngle(dTheta), false);
		}
		
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	/**
	 * 
	 * This method will stop the robot immediately.
	 */
	public void stop() {
		LEFT_MOTOR.stop(true);
		RIGHT_MOTOR.stop(false);
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	private int convertAngle(double angle) {return convertDistance(Math.PI * TRACK * angle / 360.0);}
	  
	private int convertDistance(double distance) {return (int) ((180.0 * distance) / (Math.PI * RADIUS));}
}
