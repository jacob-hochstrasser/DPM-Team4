package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab4.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
	
	//-----<Important Constants>-----//
	private final double LS_DIFF = 9.6;//cm
	private final double LS_WHEEL_DIFF = 12;//cm
	private final int INITIALIZING_SCOPE = 25;
	private final int MEASURING_SCOPE = 5;
	private final int TURNING_SPEED = 75;
	private final int LOCALIZING_SPEED = 75;
	private final int NAVIGATING_SPEED = 200;
	private final int ACCELERATION = 500;
	private final int TIME_INTERVAL = 25;//ms
	private final double TILE_SIZE = 30.48;//cm, length of one side of a tile
	private final double STANDARD_DISTANCE_FALLING = TILE_SIZE;//cm, standard distance to distinguish falling edge
	private final double NOTICE_MARGIN = 1;
	private final double ROTATION_ERROR_CW = 15;
	private double TRACK;
	private double RADIUS;
	private float BLACK_LINE_LEFT;
	private float BLACK_LINE_RIGHT;
	private int START_CORNER;// 0 -> down left, 1 -> down right, 2 -> up right, 3 -> up left
	
	//-----<Motors>-----//
	private static EV3LargeRegulatedMotor LEFT_MOTOR;
	private static EV3LargeRegulatedMotor RIGHT_MOTOR;
	
	//-----<SensorPoller>-----//
	private static SensorPoller LOCALIZING_LEFT;
	private static SensorPoller LOCALIZING_RIGHT;
	private static SensorPoller ULTRASONIC;
	
	
	public Navigation(EV3LargeRegulatedMotor LEFT_MOTOR, EV3LargeRegulatedMotor RIGHT_MOTOR, SensorPoller LOCALIZING_LEFT,
			SensorPoller LOCALIZING_RIGHT, SensorPoller ULTRASONIC, int START_CORNER, double TRACK, double RADIUS) {
		this.LEFT_MOTOR = LEFT_MOTOR;
		this.RIGHT_MOTOR = RIGHT_MOTOR;
		this.LOCALIZING_LEFT = LOCALIZING_LEFT;
		this.LOCALIZING_RIGHT = LOCALIZING_RIGHT;
		this.ULTRASONIC = ULTRASONIC;
		this.START_CORNER = START_CORNER;
		this.TRACK = TRACK;
		this.RADIUS = RADIUS;
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
	
	public void localize() {
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
		    
		    // Calculate angle of local maximum based on two detected edges and use it to find 0� 
		    double dTheta = (-45 + (pos1[2]+pos2[2])/2 + 360)%360;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
		    //Turn to 0
		    turnTo(dTheta);
		    turnLeft(ROTATION_ERROR_CW);//Fix rotational error
		    Odometer.resetTheta();//Reset theta
		    
		    boolean left = false;
		    boolean right = false;
		    double leftDetection = 0;
		    double rightDetection = 0;
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    }
			    if(detectLineRight()) {
			    	right = true;
			    	rightDetection = RIGHT_MOTOR.getTachoCount();
			    }
		    } while(left&&right);
		    stop();
		    
		    double diff = rightDetection - leftDetection;
		    dTheta = Math.toDegrees(Math.atan(diff/TRACK));
		    if(dTheta>1) {
		    	turnLeft(dTheta);
		    } else if(dTheta<-1) {
		    	turnRight(-dTheta);
		    }
		    
		    LEFT_MOTOR.rotate(-convertDistance(diff/2+LS_WHEEL_DIFF),true);
			RIGHT_MOTOR.rotate(-convertDistance(diff/2+LS_WHEEL_DIFF),false);
			
			turnRight(90);
			
			left = false;
		    right = false;
		    leftDetection = 0;
		    rightDetection = 0;
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    }
			    if(detectLineRight()) {
			    	right = true;
			    	rightDetection = RIGHT_MOTOR.getTachoCount();
			    }
		    } while(left&&right);
		    stop();
		    
		    diff = rightDetection - leftDetection;
		    dTheta = Math.toDegrees(Math.atan(diff/TRACK));
		    if(dTheta>1) {
		    	turnLeft(90+dTheta);
		    } else if(dTheta<-1) {
		    	turnLeft(90+dTheta);
		    }
			
		    stop();
		    
		    Odometer.setX(0);
		    Odometer.setY(0);
		    Odometer.setT(0);
		    
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
		    
		    // Calculate angle of local maximum based on two detected edges and use it to find 0� 
		    double dTheta = (-45 + (pos1[2]+pos2[2])/2 + 360 + 90)%360;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
		    //Turn to 0
		    turnTo(dTheta);
		    turnLeft(ROTATION_ERROR_CW);//Fix rotational error
		    Odometer.resetTheta();//Reset theta
		    
		    boolean left = false;
		    boolean right = false;
		    double leftDetection = 0;
		    double rightDetection = 0;
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    }
			    if(detectLineRight()) {
			    	right = true;
			    	rightDetection = RIGHT_MOTOR.getTachoCount();
			    }
		    } while(left&&right);
		    stop();
		    
		    double diff = rightDetection - leftDetection;
		    dTheta = Math.toDegrees(Math.atan(diff/TRACK));
		    if(dTheta>1) {
		    	turnLeft(dTheta);
		    } else if(dTheta<-1) {
		    	turnRight(-dTheta);
		    }
		    
		    LEFT_MOTOR.rotate(-convertDistance(diff/2+LS_WHEEL_DIFF),true);
			RIGHT_MOTOR.rotate(-convertDistance(diff/2+LS_WHEEL_DIFF),false);
			
			turnLeft(90);
			
			left = false;
		    right = false;
		    leftDetection = 0;
		    rightDetection = 0;
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    }
			    if(detectLineRight()) {
			    	right = true;
			    	rightDetection = RIGHT_MOTOR.getTachoCount();
			    }
		    } while(left&&right);
		    stop();
		    
		    diff = rightDetection - leftDetection;
		    dTheta = Math.toDegrees(Math.atan(diff/TRACK));
		    if(dTheta>1) {
		    	turnRight(90-dTheta);
		    } else if(dTheta<-1) {
		    	turnRight(90-dTheta);
		    }
			
		    stop();
		    
		    Odometer.setX(0);
		    Odometer.setY(0);
		    Odometer.setT(0);
		    
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
		    
		    // Calculate angle of local maximum based on two detected edges and use it to find 0� 
		    double dTheta = (-45 + (pos1[2]+pos2[2])/2 + 360)%360;
		    try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
		    
		    //Turn to 0
		    turnTo(dTheta);
		    turnLeft(ROTATION_ERROR_CW);//Fix rotational error
		    Odometer.setT(180);//Reset theta
		    
		    boolean left = false;
		    boolean right = false;
		    double leftDetection = 0;
		    double rightDetection = 0;
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    }
			    if(detectLineRight()) {
			    	right = true;
			    	rightDetection = RIGHT_MOTOR.getTachoCount();
			    }
		    } while(left&&right);
		    stop();
		    
		    double diff = rightDetection - leftDetection;
		    dTheta = Math.toDegrees(Math.atan(diff/TRACK));
		    if(dTheta>1) {
		    	turnLeft(dTheta);
		    } else if(dTheta<-1) {
		    	turnRight(-dTheta);
		    }
		    
		    LEFT_MOTOR.rotate(-convertDistance(diff/2+LS_WHEEL_DIFF),true);
			RIGHT_MOTOR.rotate(-convertDistance(diff/2+LS_WHEEL_DIFF),false);
			
			turnRight(90);
			
			left = false;
		    right = false;
		    leftDetection = 0;
		    rightDetection = 0;
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    }
			    if(detectLineRight()) {
			    	right = true;
			    	rightDetection = RIGHT_MOTOR.getTachoCount();
			    }
		    } while(left&&right);
		    stop();
		    
		    diff = rightDetection - leftDetection;
		    dTheta = Math.toDegrees(Math.atan(diff/TRACK));
		    if(dTheta>1) {
		    	turnLeft(90+dTheta);
		    } else if(dTheta<-1) {
		    	turnLeft(90+dTheta);
		    }
			
		    stop();
		    
		    Odometer.setX(0);
		    Odometer.setY(0);
		    Odometer.setT(180);
		    		    
		    
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
		    
		    // Calculate angle of local maximum based on two detected edges and use it to find 0� 
		    double dTheta = (-45 + (pos1[2]+pos2[2])/2 + 360 + 90)%360;
		    try {Thread.sleep(500);} catch (InterruptedException e1) {}
		    
		    //Turn to 0
		    turnTo(dTheta);
		    turnLeft(ROTATION_ERROR_CW);//Fix rotational error
		    Odometer.setT(180);//Reset theta
		    
		    boolean left = false;
		    boolean right = false;
		    double leftDetection = 0;
		    double rightDetection = 0;
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    }
			    if(detectLineRight()) {
			    	right = true;
			    	rightDetection = RIGHT_MOTOR.getTachoCount();
			    }
		    } while(left&&right);
		    stop();
		    
		    double diff = rightDetection - leftDetection;
		    dTheta = Math.toDegrees(Math.atan(diff/TRACK));
		    if(dTheta>1) {
		    	turnLeft(dTheta);
		    } else if(dTheta<-1) {
		    	turnRight(-dTheta);
		    }
		    
		    LEFT_MOTOR.rotate(-convertDistance(diff/2+LS_WHEEL_DIFF),true);
			RIGHT_MOTOR.rotate(-convertDistance(diff/2+LS_WHEEL_DIFF),false);
			
			turnLeft(90);
			
			left = false;
		    right = false;
		    leftDetection = 0;
		    rightDetection = 0;
		    LEFT_MOTOR.forward();
		    RIGHT_MOTOR.forward();
		    do {
			    if(detectLineLeft()) {
			    	left = true;
			    	leftDetection = LEFT_MOTOR.getTachoCount();
			    }
			    if(detectLineRight()) {
			    	right = true;
			    	rightDetection = RIGHT_MOTOR.getTachoCount();
			    }
		    } while(left&&right);
		    stop();
		    
		    diff = rightDetection - leftDetection;
		    dTheta = Math.toDegrees(Math.atan(diff/TRACK));
		    if(dTheta>1) {
		    	turnRight(90-dTheta);
		    } else if(dTheta<-1) {
		    	turnRight(90-dTheta);
		    }
			
		    stop();
		    
		    Odometer.setX(0);
		    Odometer.setY(0);
		    Odometer.setT(180);
		    		    
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
	
	 /**
	   * This method turns the robot to any given theta [0,359.9]
	   */
	  private void turnTo(double Theta) {// Theta is in the range of [0, 359.9]
		stop();
		double[] pos = Odometer.getPosition();
		double dTheta = pos[2] - Theta;
		LEFT_MOTOR.setSpeed(TURNING_SPEED);
		RIGHT_MOTOR.setSpeed(TURNING_SPEED);
		LEFT_MOTOR.setAcceleration(ACCELERATION);
		RIGHT_MOTOR.setAcceleration(ACCELERATION);
		   
		if(dTheta<-180) {
		  dTheta+=360;
		} else if(dTheta>180) {
		  dTheta-=360;
	    }
		    
		LEFT_MOTOR.rotate(convertAngle(dTheta), true);
		RIGHT_MOTOR.rotate(-convertAngle(dTheta), false);
		    
		stop();
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
	}
	
	/**
	 * This method turns the robot clockwise by a given degree
	 * @param dTheta
	 */
	private void turnLeft(double dTheta) {
		stop();
		LEFT_MOTOR.setSpeed(TURNING_SPEED);
		RIGHT_MOTOR.setSpeed(TURNING_SPEED);
		LEFT_MOTOR.setAcceleration(ACCELERATION);
		RIGHT_MOTOR.setAcceleration(ACCELERATION);
		
		try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
		
		if (dTheta<0) {
			turnRight(-dTheta);
		} else {
		    LEFT_MOTOR.rotate(convertAngle(dTheta), true);
		    RIGHT_MOTOR.rotate(-convertAngle(dTheta), false);
		}
	}
	  
	/**
	 * This method turns the robot counter clockwise by a given degree
	 * @param dTheta
	 */
	private void turnRight(double dTheta) {
		stop();
		LEFT_MOTOR.setSpeed(TURNING_SPEED);
		RIGHT_MOTOR.setSpeed(TURNING_SPEED);
		LEFT_MOTOR.setAcceleration(ACCELERATION);
		RIGHT_MOTOR.setAcceleration(ACCELERATION);
		
		try {Thread.sleep(100);} catch (InterruptedException e) {}
		
		if (dTheta<0) {
		    turnLeft(-dTheta);
		} else {
		    LEFT_MOTOR.rotate(-convertAngle(dTheta), true);
		    RIGHT_MOTOR.rotate(convertAngle(dTheta), false);
		}
	}
	  
    /**
	 * This method converts an angle in degree to number of rotation of motors
	 * @param angle
	 * @return
	 */
	private int convertAngle(double angle) {return convertDistance(Math.PI * TRACK * angle / 360.0);}
	  
	/**
	 * This method converts a distance in cm to number of rotation of motors
	 * @param distance
	 * @return
	 */
	private int convertDistance(double distance) {return (int) ((180.0 * distance) / (Math.PI * RADIUS));}
	
	private boolean detectLineLeft() {
		float leftData = LOCALIZING_LEFT.getData(MEASURING_SCOPE);
		return 1.15 < leftData/BLACK_LINE_LEFT || 0.85 > leftData/BLACK_LINE_LEFT;
	}
	
	private boolean detectLineRight() {
		float rightData = LOCALIZING_LEFT.getData(MEASURING_SCOPE);
		return 1.15 < rightData/BLACK_LINE_RIGHT || 0.85 > rightData/BLACK_LINE_RIGHT;
	}
	
	
	
	public void demo() {
		initialize();
		LEFT_MOTOR.setSpeed(LOCALIZING_SPEED);
		RIGHT_MOTOR.setSpeed(LOCALIZING_SPEED);
		try {Thread.sleep(100);} catch (InterruptedException e) {}
		LEFT_MOTOR.forward();
		RIGHT_MOTOR.forward();
		do {
			float leftData = LOCALIZING_LEFT.getData(MEASURING_SCOPE);
			float rightData = LOCALIZING_RIGHT.getData(MEASURING_SCOPE);
			float lRate = leftData/BLACK_LINE_LEFT;
			float rRate = rightData/BLACK_LINE_RIGHT;
			boolean result1 = 1.15 < lRate || 0.85 > lRate;
			boolean result2 = 1.15 < rRate || 0.85 > rRate;
			System.out.println("STD = " + BLACK_LINE_LEFT + "\nRatio = " + lRate);
			if(result1) {
				Sound.beep();
				LEFT_MOTOR.stop();
			}
			if(result2) {
				Sound.twoBeeps();
				RIGHT_MOTOR.stop();
			}
			if(!LEFT_MOTOR.isMoving()&&!RIGHT_MOTOR.isMoving()) {
				break;
			}
		} while(true);
		/*
		initialize();
		LEFT_MOTOR.setSpeed(NAVIGATING_SPEED);
		RIGHT_MOTOR.setSpeed(NAVIGATING_SPEED);
		boolean[] detection = detectLine();
		do {
		LEFT_MOTOR.forward();
		RIGHT_MOTOR.forward();
		detection = detectLine();
		} while(!detection[2]);
		LEFT_MOTOR.stop(true);
		RIGHT_MOTOR.stop(false);
		*/
	}
}