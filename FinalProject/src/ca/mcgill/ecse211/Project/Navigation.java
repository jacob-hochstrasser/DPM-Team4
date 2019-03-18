package ca.mcgill.ecse211.Project;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
	/** 
	 * This is a class that controlls motors. Many of the movements could be find here.
	 * 
	 * @author Pengnan Fan
	 *
	 */
	
	//-----<Important Constant>-----//
	public static final double RADIUS = 1.6;//cm. It is certifed by testing from Pengnan.
	public static final double TRACK = 24.5;//cm. It is certified by testing from Pengnan.
	volatile private static int ACCELERATION = 1200;
	volatile private static int SPEED = 500;
	private static final int TIME_INTERVAL = 15;//ms
	private static final double TILE_SIZE = 30.28;//cm 
	private static final double EDGE_DISTANCE = 1.5*TILE_SIZE;
	private static final double NOTICE_MARGIN = 1;//cm
	private static final double LS_TK_DIS = 8.75;//cm
	private static final double LS_DIFF = TRACK;
	private static final int INITIALIZING_SCOPE = 25;
	private static final int MEASURING_SCOPE = 5;
	private final double ROTATION_ERROR_CW = 40;
	private float LINE_L;
	private float LINE_R;
	
	//-----<Searching Coordinates and Starting Corner>-----//
	private int START_CORNER = 0;// 0 -> down left, 1 -> down right, 2 -> up right, 3 -> up left
	private int[] LL;
	private int[] UR;
		
	//-----<Motor>-----//
	public static EV3LargeRegulatedMotor LEFT_MOTOR;
	public static EV3LargeRegulatedMotor RIGHT_MOTOR;
	
	//-----<Sensors>-----//
	private static SensorPoller LS_L = new LightSensorPoller("S2", TIME_INTERVAL, false);
	private static SensorPoller LS_R = new LightSensorPoller("S1", TIME_INTERVAL, false);
	private static SensorPoller US = new UltrasonicSensorPoller("S3", TIME_INTERVAL);
	
	//-----<Position>-----//
	private float[] position = new float[] {0,0,0};
	
	//-----<Nested MotorController>-----//
	private static Navigation NV;
	
	public void demo() {
		localize();
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This is a static method used to get a Navigation object.
	 * Note: Only one of this type of object can be accessed.
	 * 
	 * @param leftMotor:String
	 * This input shows the port used for left motor.
	 * 
	 * @param rightMotor:String
	 * This input shows the port used for right motor.
	 * 
	 * @return NV:Navigation
	 * This output is the Navigation nested in the class.
	 * 
	 */
	public static Navigation getNavigation(String leftMotor, String rightMotor) {
		if(NV==null) {
			NV = new Navigation(leftMotor, rightMotor);
		}
		return NV;
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method sets up searching range and starting corner
	 * 
	 * @param LL:int array
	 * This input suggests the lower left coordinates of the searching range
	 * 
	 * @param UR:int array
	 * This input suggests the upper right coordinates of the searching range
	 * 
	 * @param START_CORNER
	 * This input suggests the starting corner
	 * 
	 */
	public void setup(int[] LL, int[] UR, int START_CORNER) {
		this.LL = LL;
		this.UR = UR;
		this.START_CORNER = START_CORNER;
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
	 * @author Pengnan Fan
	 * 
	 * This drives the robot to a given coordinates
	 * 
	 * @param x: in the unit of tile size
	 * @param y: in the unit of tile size
	 */
	public void travelTo(double x, double y) {
		x*=TILE_SIZE;
		y*=TILE_SIZE;
		double dX = x - position[0];
		double dY = y - position[1];
		
		turnTo(calculateTheta(dX, dY));
		
		double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
		
		moveForward(distance);

	    //Sound.beep();
	}
	
	public void travelTo(int[] coords) {
	    double x = coords[0];
	    double y = coords[1];
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
	 * @author Pengnan Fan
	 * 
	 * This method reset the acceleration and re-initialize the motor.
	 * 
	 * @param newAcceleration:int
	 * This input is the new acceleration.
	 * 
	 */
	public void setAcceleration(int newAcceleration) {
		ACCELERATION = newAcceleration;
		initializeAcceleration();
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method reset the speed and re-initialize the motor.
	 * 
	 * @param newSpeed:int
	 * This input is the new speed.
	 * 
	 */
	public void setSpeed(int newSpeed) {
		SPEED = newSpeed;
		initializeSpeed();
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method initialize the speed of motors.
	 * 
	 */
	public void initializeSpeed() {
		synchronized(EV3LargeRegulatedMotor.class) {
			LEFT_MOTOR.setSpeed(SPEED);
			RIGHT_MOTOR.setSpeed(SPEED);
		}
	}

	/**
	 * @author Pengnan Fan
	 * 
	 * This method initialize the acceleration of motors.
	 * 
	 */
	public void initializeAcceleration() {
		synchronized(EV3LargeRegulatedMotor.class) {
			LEFT_MOTOR.setAcceleration(ACCELERATION);
			RIGHT_MOTOR.setAcceleration(ACCELERATION);
		}
	}
	
	private boolean detectLineLeft() {
		float leftData = LS_L.getData(MEASURING_SCOPE);
		return 1.15 < leftData/LINE_L || 0.85 > leftData/LINE_L;
	}
	
	private boolean detectLineRight() {
		float rightData = LS_R.getData(MEASURING_SCOPE);
		return 1.15 < rightData/LINE_R || 0.85 > rightData/LINE_R;
	}

	
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
		    Odometer.setX(TILE_SIZE);
		    Odometer.setY(TILE_SIZE);
		    Odometer.setT(0);
		    position = new float[] {1,1,0};
		    
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
		    position = new float[] {7,1,0};
		    
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
		    position = new float[] {7,7,180};
		    
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
		    position = new float[] {1,7,0};
		    		    
		} else {
			Sound.beep();
			Sound.beep();
			Sound.beep();
			System.exit(0);
		}
		Sound.beep();
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method drives the robot backward to a given distance. It will automatically call moveForward if the distance is negative
	 * 
	 * @param distance:int
	 * This input shows how much the robot should move backward in centimeter.
	 * 
	 */
	public void moveBackward(double distance) {
		if(distance<0) {moveForward(distance);}
			
		synchronized(EV3LargeRegulatedMotor.class) {
			LEFT_MOTOR.rotate(-convertDistance(distance), true);
			RIGHT_MOTOR.rotate(-convertDistance(distance), false);
		}
			
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method drives the robot forward to a given distance. It will automatically call moveBackward if the distance is negative.
	 * 
	 * @param distance:int
	 * This input shows how much the robot should move forward in centimeter.
	 * 
	 */
	public void moveForward(double distance) {
		if(distance<0) {moveBackward(distance);}
		
		synchronized(EV3LargeRegulatedMotor.class) {
			LEFT_MOTOR.rotate(convertDistance(distance), true);
			RIGHT_MOTOR.rotate(convertDistance(distance), false);
		}
		
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
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
	
	/**
	 * @author Pengnan Fan
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
			synchronized(EV3LargeRegulatedMotor.class) {
				LEFT_MOTOR.rotate(-convertAngle(dTheta), true);
			    RIGHT_MOTOR.rotate(convertAngle(dTheta), false);
			}
		}
		
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	/**
	 * @author Pengnan Fan
	 * 
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
			synchronized(EV3LargeRegulatedMotor.class) {
				LEFT_MOTOR.rotate(convertAngle(dTheta), true);
			    RIGHT_MOTOR.rotate(-convertAngle(dTheta), false);
			}
		}
		
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	/**
	 * @author Pengnan Fan
	 * 
	 * This method will stop the robot immediately.
	 */
	public void stop() {
		synchronized(EV3LargeRegulatedMotor.class) {
			LEFT_MOTOR.stop(true);
			RIGHT_MOTOR.stop(false);
		}
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	private int convertAngle(double angle) {return convertDistance(Math.PI * TRACK * angle / 360.0);}
	  
	private int convertDistance(double distance) {return (int) ((180.0 * distance) / (Math.PI * RADIUS));}
}
