package ca.mcgill.ecse211.Project;


import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/** 
 * This is a class that controls the movements of the Lego EV3 robot.
 * 
 * @author Pengnan, Jake
 *
 */

public class Navigation {
	
	//-----<Important Constant>-----//
	/**
	 * This parameter stands for the radius of wheels.
	 */
	public static final double RADIUS = 2.1;//cm. It is certifed by testing from Pengnan.
	/**
	 * This parameter stands for the distances between two tracks.
	 */
	public static final double TRACK = 13.45;//cm. It is certified by testing from Pengnan.
	/**
	 * Angular acceleration of the motors
	 */
	volatile private static int ACCELERATION = 1200;
	/**
	 * Angular velocity of the motors
	 */
	volatile private static int SPEED = 450;
	/**
	 * Time interval used for thread timing
	 */
	private static final int TIME_INTERVAL = 15;//ms
	private static final int MEASURING_INTERVAL = 0;
	/**
	 * Size of floor tiles
	 */
	private static final double TILE_SIZE = 30.28;//cm
	
	private static final double FACTOR = 1;
	
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
	private static final double LS_TK_DIS = 9;//cm
	/**
	 * Distance between the two light sensors
	 */
	private static final double LS_DIFF = 16;
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
	private static final double ROTATION_ERROR = 0;
	
	private static final long HEAVY = 500;
	
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
	private int[] SEARCH_LL;
	/**
	 * The upper right corner of the table
	 */
	private int[] SEARCH_UR;
		
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
	private static SensorPoller LS_L = new LightSensorPoller("S1", MEASURING_INTERVAL, false);
	/**
	 * Sensor poller for the right localizing light sensor; acts like a wrapper for the sensor itself
	 */
	private static SensorPoller LS_R = new LightSensorPoller("S2", MEASURING_INTERVAL, false);
	/**
	 * Sensor poller for the ultrasonic sensor; acts like a wrapper for the sensor itself
	 */
	private static SensorPoller US = new UltrasonicSensorPoller("S3", MEASURING_INTERVAL);
	
	private static int[] AREA_LL;
	private static int[] AREA_UR;
	private static int[] ISLAND_LL;
	private static int[] ISLAND_UR;
	private static int[] TUNNEL_LL;
	private static int[] TUNNEL_UR;
	
	//-----<Position>-----//
	/**
	 * 
	 */
	private double[] position = new double[] {0,0,0};
	
	//-----<Nested MotorController>-----//
	private static Navigation NV;
	
	//private static Identifier ID = BetaDemoApplication.ID;
	
	private boolean foundCan = false;
	
	protected static final boolean DEBUG = false;
	
	/**
	 * This is only for block testing
	 */
	void demo() {
		//double data = US.getData();
		//setSpeed(100);
		//LEFT_MOTOR.rotate(convertAngle(90), true);
		//RIGHT_MOTOR.rotate(-convertAngle(90), true);
		lightLocLine();
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
	public void setup(int[] LL, int[] UR, int startCorner, int[] area_ll, int[] area_ur, int[] island_ll, int[] island_ur, int[] tunnel_ll, int[] tunnel_ur) {
		this.SEARCH_LL = LL;
		this.SEARCH_UR = UR;
		this.START_CORNER = startCorner;
		this.AREA_LL = area_ll;
		this.AREA_UR = area_ur;
		this.ISLAND_LL = island_ll;
		this.ISLAND_UR = island_ur;
		this.TUNNEL_LL = tunnel_ll;
		this.TUNNEL_UR = tunnel_ur;
		initializeAcceleration();
		initializeSpeed();
	}
	
	private boolean isInTheRange(int[] area_LL, int[] area_UR, double[] nextPos) {
		return (nextPos[0] >= area_LL[0]) && (nextPos[0] <= area_UR[0]) && (nextPos[1] >= area_LL[1]) && (nextPos[1] <= area_UR[1]);
	}
	
	private boolean isInTheRange(int[] area_LL, int[] area_UR, int[] nextPos) {
		return (nextPos[0] >= area_LL[0]) && (nextPos[0] <= area_UR[0]) && (nextPos[1] >= area_LL[1]) && (nextPos[1] <= area_UR[1]);
	}
	
	private boolean isTunnelVertical() {
		int[] tunnel_LR = new int[] {TUNNEL_UR[0], TUNNEL_LL[1]};
		return isInTheRange(AREA_LL, AREA_UR, TUNNEL_LL) && isInTheRange(AREA_LL, AREA_UR, tunnel_LR);
	}
	
	private int isTunnelCloseToWalls() {
		//Return number to indicate walls.
		// 0 -> Down
		// 1 -> Right
		// 2 -> Up
		// 3 -> Left
		// -1 -> Not closed to walls
		if(TUNNEL_LL[1] == 0) {
			//close to down wall
			return 0;
		} else if(TUNNEL_UR[0] == 15) {
			//close to right wall
			return 1;
		} else if(TUNNEL_UR[1] == 9){
			//close to up wall
			return 2;
		} else if(TUNNEL_LL[0] == 0) {
			//close to left wall
			return 3;
		}
		return -1;
	}
	
	private int isTunnelNearWater(boolean forward, boolean vertical) {
		//Return number to indicate water.
		// 0 -> Down
		// 1 -> Right
		// 2 -> Up
		// 3 -> Left
		// -1 -> Not closed to water
		if(forward) {
			if(vertical) {
				if(TUNNEL_LL[0] == AREA_LL[0]) {
					return 3;
				} else if(TUNNEL_UR[0] == AREA_UR[0]) {
					return 1;
				}
			} else {
				if(TUNNEL_LL[1] == AREA_LL[1]) {
					return 0;
				} else if(TUNNEL_UR[1] == AREA_UR[1]) {
					return 2;
				}
			}
		} else {
			if(vertical) {
				if(TUNNEL_LL[0] == ISLAND_LL[0]) {
					return 3;
				} else if(TUNNEL_UR[0] == ISLAND_UR[0]) {
					return 1;
				}
			} else {
				if(TUNNEL_LL[1] == ISLAND_LL[1]) {
					return 0;
				} else if(TUNNEL_UR[1] == ISLAND_UR[1]) {
					return 2;
				}
			}
		}
		return -1;
	}
	
	private void lightLocLine() {
		double dTheta, diff;
        boolean left = false, right = false;
        double leftDetection = 0, rightDetection = 0;
        
        int speed = SPEED;
        setSpeed(200);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.forward();
        do {
        	if(detectLineLeft()&&!left) {
            	left = true;
            	leftDetection = LEFT_MOTOR.getTachoCount();
            	if(right) {
            		stop();
            	}
            }
            if(detectLineRight()&&!right) {
            	right = true;
            	rightDetection = LEFT_MOTOR.getTachoCount();
            	if(left) {
            		stop();
            	}
            }
        } while(!left||!right);
        //setSpeed(speed);
        
        diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
        dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
        
        turnLeft(dTheta);
        setSpeed(SPEED);
	}
	
	private boolean leftToRightOrUpToDownAsForward() {
		return isInTheRange(AREA_LL, AREA_UR, TUNNEL_LL);
	}
	
	public void passTheTunnel(boolean forward) {
		boolean vertical = isTunnelVertical();
		int closeToWall = isTunnelCloseToWalls();
		int closeToWater_area = isTunnelNearWater(true, vertical);
		int closeToWater_island = isTunnelNearWater(false, vertical);
		if(leftToRightOrUpToDownAsForward()) {
			if(forward) {
				//Going from starting point to the island
				if(vertical) {
					//Vertical tunnel
					if(closeToWall == -1) {
						//Not close to wall
						if (closeToWater_area == -1||closeToWater_area == 1) {
							//Not close to water or close to right water
							travelTo(TUNNEL_LL[0], TUNNEL_LL[1] - 1);
							turnTo(0);
							reLocalize(TUNNEL_LL[0], TUNNEL_LL[1] - 1);
							turnTo(90);
							//moveForward(TILE_SIZE/2);
							//turnTo(0);
							//moveForward(((TUNNEL_UR[1] - TUNNEL_LL[1]) + 1)*TILE_SIZE);
						} else if(closeToWater_area == 3) {
							//Close to left water
							travelTo(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
							turnTo(0);
							reLocalize(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
							turnTo(270);
	                        //moveForward(TILE_SIZE/2);
	                        //turnTo(0);
	                        //moveForward(((TUNNEL_UR[1] - TUNNEL_LL[1]) + 1)*TILE_SIZE);
						}
					} else if(closeToWall == 1) {
						travelTo(TUNNEL_LL[0], TUNNEL_LL[1] - 1);
						turnTo(0);
						reLocalize(TUNNEL_LL[0], TUNNEL_LL[1] - 1);
						turnTo(90);
	                    //moveForward(TILE_SIZE/2);
	                    //turnTo(0);
	                    //moveForward(((TUNNEL_UR[1] - TUNNEL_LL[1]) + 1)*TILE_SIZE);
					} else if(closeToWall == 3) {
						travelTo(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
						turnTo(0);
						reLocalize(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
						turnTo(270);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(0);
	                    //moveForward(((TUNNEL_UR[1] - TUNNEL_LL[1]) + 1)*TILE_SIZE);
					}
					moveForward(TILE_SIZE/2);
					turnTo(0);
					lightLocLine();
					travelTo(TUNNEL_LL[0]+0.5, TUNNEL_UR[1]+1);
				} else {
					//Horizontal
					if(closeToWall == -1) {
						//Not close to wall
						if (closeToWater_area == -1||closeToWater_area == 0) {
							//Not close to water or close to right water
							travelTo(TUNNEL_LL[0] - 1, TUNNEL_LL[1] + 1);
							turnTo(0);
							reLocalize(TUNNEL_LL[0] - 1, TUNNEL_LL[1] + 1);
							turnTo(180);
	                        //moveForward(TILE_SIZE/2);
	                        //turnTo(90);
	                        //moveForward(((TUNNEL_UR[0] - TUNNEL_LL[0]) + 1)*TILE_SIZE);
						} else if(closeToWater_area == 2) {
							//Close to left water
							travelTo(TUNNEL_LL[0] - 1, TUNNEL_LL[1]);
							turnTo(0);
							reLocalize(TUNNEL_LL[0] - 1, TUNNEL_LL[1]);
	                        //moveForward(TILE_SIZE/2);
	                        //turnTo(90);
	                        //moveForward(((TUNNEL_UR[0] - TUNNEL_LL[0]) + 1)*TILE_SIZE);
						}
					} else if(closeToWall == 0) {
						travelTo(TUNNEL_LL[0] - 1, TUNNEL_LL[1] + 1);
						turnTo(0);
						reLocalize(TUNNEL_LL[0] - 1, TUNNEL_LL[1] + 1);
						turnTo(180);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(90);
	                    //moveForward(((TUNNEL_UR[0] - TUNNEL_LL[0]) + 1)*TILE_SIZE);
					} else if(closeToWall == 2) {
						travelTo(TUNNEL_LL[0] - 1, TUNNEL_LL[1]);
						turnTo(0);
						reLocalize(TUNNEL_LL[0] - 1, TUNNEL_LL[1]);
						//turnTo(45);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(90);
	                    //moveForward(((TUNNEL_UR[0] - TUNNEL_LL[0]) + 1)*TILE_SIZE);
					}
					moveForward(TILE_SIZE/2);
	                turnTo(90);
					lightLocLine();
					travelTo(TUNNEL_UR[0]+1, TUNNEL_UR[1]-0.5);
				}
				
				//Passed Tunnel
				if(closeToWater_island == 0||closeToWall == 0) {
					travelTo(TUNNEL_UR[0] + 1, TUNNEL_UR[1]);
					reLocalize(TUNNEL_UR[0] + 1, TUNNEL_UR[1]);
				} else if(closeToWater_island == 1||closeToWall == 1) {
					travelTo(TUNNEL_UR[0] - 1, TUNNEL_UR[1] + 1);
					reLocalize(TUNNEL_UR[0] - 1, TUNNEL_UR[1] + 1);
				} else if(closeToWater_island == 2||closeToWall == 2) {
					travelTo(TUNNEL_UR[0] + 1, TUNNEL_UR[1] - 1);
					reLocalize(TUNNEL_UR[0] + 1, TUNNEL_UR[1] - 1);
				} else if(closeToWater_island == 3||closeToWall == 3) {
					travelTo(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
					reLocalize(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
				} else {
					travelTo(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
					reLocalize(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
				}
			} else {
				//Going back from the searching LL to the starting area
				if(vertical) {
					//Vertical tunnel
					if(closeToWall == -1) {
						//Not close to wall
						if (closeToWater_island == -1||closeToWater_island == 1) {
							//Not close to water or close to right water
							travelTo(TUNNEL_UR[0] - 1, TUNNEL_UR[1] + 1);
							turnTo(0);
							reLocalize(TUNNEL_UR[0] - 1, TUNNEL_UR[1] + 1);
							turnTo(90);
							//moveForward(TILE_SIZE*Math.sqrt(2)/2);
							//turnTo(180);
							//moveForward((TUNNEL_UR[1] - TUNNEL_LL[1] +1)*TILE_SIZE);
						} else if(closeToWater_island == 3) {
							//Close to left water
							travelTo(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
							turnTo(0);
							reLocalize(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
							turnTo(270);
	                        //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                        //turnTo(180);
	                        //moveForward((TUNNEL_UR[1] - TUNNEL_LL[1] +1)*TILE_SIZE);
						}
					} else if(closeToWall == 1) {
						travelTo(TUNNEL_UR[0] - 1, TUNNEL_UR[1] + 1);
						turnTo(0);
						reLocalize(TUNNEL_UR[0] - 1, TUNNEL_UR[1] + 1);
						turnTo(90);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(180);
	                    //moveForward((TUNNEL_UR[1] - TUNNEL_LL[1] +1)*TILE_SIZE);
					} else if(closeToWall == 3) {
						travelTo(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
						turnTo(0);
						reLocalize(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
						turnTo(270);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(180);
	                    //moveForward((TUNNEL_UR[1] - TUNNEL_LL[1] +1)*TILE_SIZE);
					}
					moveForward(TILE_SIZE/2);
	                turnTo(180);
					lightLocLine();
					travelTo(TUNNEL_LL[0]+0.5, TUNNEL_LL[1]-1);
				} else {
					//Horizontal
					if(closeToWall == -1) {
						//Not close to wall
						if (closeToWater_island == -1||closeToWater_island == 0) {
							//Not close to water or close to right water
							travelTo(TUNNEL_UR[0] + 1, TUNNEL_UR[1]);
							turnTo(0);
							reLocalize(TUNNEL_UR[0] + 1, TUNNEL_UR[1]);
							turnTo(180);
	                        //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                        //turnTo(270);
	                        //moveForward((TUNNEL_UR[0] - TUNNEL_LL[0] +1)*TILE_SIZE);
						} else if(closeToWater_island == 2) {
							//Close to left water
							travelTo(TUNNEL_UR[0] + 1, TUNNEL_UR[1] - 1);
							turnTo(0);
							reLocalize(TUNNEL_UR[0] + 1, TUNNEL_UR[1] - 1);
							//turnTo(315);
	                        //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                        //turnTo(270);
	                        //moveForward((TUNNEL_UR[0] - TUNNEL_LL[0] +1)*TILE_SIZE);
						}
					} else if(closeToWall == 0) {
						travelTo(TUNNEL_UR[0] + 1, TUNNEL_UR[1]);
						turnTo(0);
						reLocalize(TUNNEL_UR[0] + 1, TUNNEL_UR[1]);
						turnTo(180);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(270);
	                    //moveForward((TUNNEL_UR[0] - TUNNEL_LL[0] +1)*TILE_SIZE);
					} else if(closeToWall == 2) {
						travelTo(TUNNEL_UR[0] + 1, TUNNEL_UR[1] - 1);
						turnTo(0);
						reLocalize(TUNNEL_UR[0] + 1, TUNNEL_UR[1] - 1);
						//turnTo(315);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(270);
	                    //moveForward((TUNNEL_UR[0] - TUNNEL_LL[0] +1)*TILE_SIZE);
					}
					moveForward(TILE_SIZE/2);
	                turnTo(270);
					lightLocLine();
					travelTo(TUNNEL_LL[0]-1, TUNNEL_LL[1]+0.5);
				}
				
				//Passed Tunnel
				if(closeToWater_area == 0||closeToWall == 0) {
					travelTo(TUNNEL_LL[0] - 1, TUNNEL_LL[1] + 1);
					reLocalize(TUNNEL_LL[0] - 1, TUNNEL_LL[1] + 1);
				} else if(closeToWater_area == 1||closeToWall == 1) {
					travelTo(TUNNEL_LL[0], TUNNEL_LL[1] - 1);
					reLocalize(TUNNEL_LL[0], TUNNEL_LL[1] - 1);
				} else if(closeToWater_area == 2||closeToWall == 2) {
					travelTo(TUNNEL_LL[0] - 1, TUNNEL_LL[1]);
					reLocalize(TUNNEL_LL[0] - 1, TUNNEL_LL[1]);
				} else if(closeToWater_area == 3||closeToWall == 3) {
					travelTo(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
					reLocalize(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
				} else {
					travelTo(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
					reLocalize(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
				}
			}
		} else {
			//Right to left as forward
			if(!forward) {
				//Going from starting point to the island
				if(vertical) {
					//Vertical tunnel
					if(closeToWall == -1) {
						//Not close to wall
						if (closeToWater_area == -1||closeToWater_area == 1) {
							//Not close to water or close to right water
							travelTo(TUNNEL_LL[0], TUNNEL_LL[1] - 1);
							turnTo(0);
							reLocalize(TUNNEL_LL[0], TUNNEL_LL[1] - 1);
							turnTo(90);
							//moveForward(TILE_SIZE/2);
							//turnTo(0);
							//moveForward(((TUNNEL_UR[1] - TUNNEL_LL[1]) + 1)*TILE_SIZE);
						} else if(closeToWater_area == 3) {
							//Close to left water
							travelTo(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
							turnTo(0);
							reLocalize(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
							turnTo(270);
	                        //moveForward(TILE_SIZE/2);
	                        //turnTo(0);
	                        //moveForward(((TUNNEL_UR[1] - TUNNEL_LL[1]) + 1)*TILE_SIZE);
						}
					} else if(closeToWall == 1) {
						travelTo(TUNNEL_LL[0], TUNNEL_LL[1] - 1);
						turnTo(0);
						reLocalize(TUNNEL_LL[0], TUNNEL_LL[1] - 1);
						turnTo(90);
	                    //moveForward(TILE_SIZE/2);
	                    //turnTo(0);
	                    //moveForward(((TUNNEL_UR[1] - TUNNEL_LL[1]) + 1)*TILE_SIZE);
					} else if(closeToWall == 3) {
						travelTo(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
						turnTo(0);
						reLocalize(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
						turnTo(270);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(0);
	                    //moveForward(((TUNNEL_UR[1] - TUNNEL_LL[1]) + 1)*TILE_SIZE);
					}
					moveForward(TILE_SIZE/2);
					turnTo(0);
					lightLocLine();
					travelTo(TUNNEL_LL[0]+0.5, TUNNEL_UR[1]+1);
				} else {
					//Horizontal
					if(closeToWall == -1) {
						//Not close to wall
						if (closeToWater_area == -1||closeToWater_area == 0) {
							//Not close to water or close to right water
							travelTo(TUNNEL_LL[0] - 1, TUNNEL_LL[1] + 1);
							turnTo(0);
							reLocalize(TUNNEL_LL[0] - 1, TUNNEL_LL[1] + 1);
							turnTo(180);
	                        //moveForward(TILE_SIZE/2);
	                        //turnTo(90);
	                        //moveForward(((TUNNEL_UR[0] - TUNNEL_LL[0]) + 1)*TILE_SIZE);
						} else if(closeToWater_area == 2) {
							//Close to left water
							travelTo(TUNNEL_LL[0] - 1, TUNNEL_LL[1]);
							turnTo(0);
							reLocalize(TUNNEL_LL[0] - 1, TUNNEL_LL[1]);
	                        //moveForward(TILE_SIZE/2);
	                        //turnTo(90);
	                        //moveForward(((TUNNEL_UR[0] - TUNNEL_LL[0]) + 1)*TILE_SIZE);
						}
					} else if(closeToWall == 0) {
						travelTo(TUNNEL_LL[0] - 1, TUNNEL_LL[1] + 1);
						turnTo(0);
						reLocalize(TUNNEL_LL[0] - 1, TUNNEL_LL[1] + 1);
						turnTo(180);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(90);
	                    //moveForward(((TUNNEL_UR[0] - TUNNEL_LL[0]) + 1)*TILE_SIZE);
					} else if(closeToWall == 2) {
						travelTo(TUNNEL_LL[0] - 1, TUNNEL_LL[1]);
						turnTo(0);
						reLocalize(TUNNEL_LL[0] - 1, TUNNEL_LL[1]);
						//turnTo(45);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(90);
	                    //moveForward(((TUNNEL_UR[0] - TUNNEL_LL[0]) + 1)*TILE_SIZE);
					}
					moveForward(TILE_SIZE/2);
	                turnTo(90);
					lightLocLine();
					travelTo(TUNNEL_UR[0]+1, TUNNEL_UR[1]-0.5);
				}
				
				//Passed Tunnel
				if(closeToWater_island == 0||closeToWall == 0) {
					travelTo(TUNNEL_UR[0] + 1, TUNNEL_UR[1]);
					reLocalize(TUNNEL_UR[0] + 1, TUNNEL_UR[1]);
				} else if(closeToWater_island == 1||closeToWall == 1) {
					travelTo(TUNNEL_UR[0] - 1, TUNNEL_UR[1] + 1);
					reLocalize(TUNNEL_UR[0] - 1, TUNNEL_UR[1] + 1);
				} else if(closeToWater_island == 2||closeToWall == 2) {
					travelTo(TUNNEL_UR[0] + 1, TUNNEL_UR[1] - 1);
					reLocalize(TUNNEL_UR[0] + 1, TUNNEL_UR[1] - 1);
				} else if(closeToWater_island == 3||closeToWall == 3) {
					travelTo(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
					reLocalize(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
				} else {
					travelTo(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
					reLocalize(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
				}
			} else {
				//Going back from the searching LL to the starting area
				if(vertical) {
					//Vertical tunnel
					if(closeToWall == -1) {
						//Not close to wall
						if (closeToWater_island == -1||closeToWater_island == 1) {
							//Not close to water or close to right water
							travelTo(TUNNEL_UR[0] - 1, TUNNEL_UR[1] + 1);
							turnTo(0);
							reLocalize(TUNNEL_UR[0] - 1, TUNNEL_UR[1] + 1);
							turnTo(90);
							//moveForward(TILE_SIZE*Math.sqrt(2)/2);
							//turnTo(180);
							//moveForward((TUNNEL_UR[1] - TUNNEL_LL[1] +1)*TILE_SIZE);
						} else if(closeToWater_island == 3) {
							//Close to left water
							travelTo(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
							turnTo(0);
							reLocalize(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
							turnTo(270);
	                        //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                        //turnTo(180);
	                        //moveForward((TUNNEL_UR[1] - TUNNEL_LL[1] +1)*TILE_SIZE);
						}
					} else if(closeToWall == 1) {
						travelTo(TUNNEL_UR[0] - 1, TUNNEL_UR[1] + 1);
						turnTo(0);
						reLocalize(TUNNEL_UR[0] - 1, TUNNEL_UR[1] + 1);
						turnTo(90);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(180);
	                    //moveForward((TUNNEL_UR[1] - TUNNEL_LL[1] +1)*TILE_SIZE);
					} else if(closeToWall == 3) {
						travelTo(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
						turnTo(0);
						reLocalize(TUNNEL_UR[0], TUNNEL_UR[1] + 1);
						turnTo(270);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(180);
	                    //moveForward((TUNNEL_UR[1] - TUNNEL_LL[1] +1)*TILE_SIZE);
					}
					moveForward(TILE_SIZE/2);
	                turnTo(180);
					lightLocLine();
					travelTo(TUNNEL_LL[0]+0.5, TUNNEL_LL[1]-1);
				} else {
					//Horizontal
					if(closeToWall == -1) {
						//Not close to wall
						if (closeToWater_island == -1||closeToWater_island == 0) {
							//Not close to water or close to right water
							travelTo(TUNNEL_UR[0] + 1, TUNNEL_UR[1]);
							turnTo(0);
							reLocalize(TUNNEL_UR[0] + 1, TUNNEL_UR[1]);
							turnTo(180);
	                        //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                        //turnTo(270);
	                        //moveForward((TUNNEL_UR[0] - TUNNEL_LL[0] +1)*TILE_SIZE);
						} else if(closeToWater_island == 2) {
							//Close to left water
							travelTo(TUNNEL_UR[0] + 1, TUNNEL_UR[1] - 1);
							turnTo(0);
							reLocalize(TUNNEL_UR[0] + 1, TUNNEL_UR[1] - 1);
							//turnTo(315);
	                        //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                        //turnTo(270);
	                        //moveForward((TUNNEL_UR[0] - TUNNEL_LL[0] +1)*TILE_SIZE);
						}
					} else if(closeToWall == 0) {
						travelTo(TUNNEL_UR[0] + 1, TUNNEL_UR[1]);
						turnTo(0);
						reLocalize(TUNNEL_UR[0] + 1, TUNNEL_UR[1]);
						turnTo(180);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(270);
	                    //moveForward((TUNNEL_UR[0] - TUNNEL_LL[0] +1)*TILE_SIZE);
					} else if(closeToWall == 2) {
						travelTo(TUNNEL_UR[0] + 1, TUNNEL_UR[1] - 1);
						turnTo(0);
						reLocalize(TUNNEL_UR[0] + 1, TUNNEL_UR[1] - 1);
						//turnTo(315);
	                    //moveForward(TILE_SIZE*Math.sqrt(2)/2);
	                    //turnTo(270);
	                    //moveForward((TUNNEL_UR[0] - TUNNEL_LL[0] +1)*TILE_SIZE);
					}
					moveForward(TILE_SIZE/2);
	                turnTo(270);
					lightLocLine();
					travelTo(TUNNEL_LL[0]-1, TUNNEL_LL[1]+0.5);
				}
				
				//Passed Tunnel
				if(closeToWater_area == 0||closeToWall == 0) {
					travelTo(TUNNEL_LL[0] - 1, TUNNEL_LL[1] + 1);
					reLocalize(TUNNEL_LL[0] - 1, TUNNEL_LL[1] + 1);
				} else if(closeToWater_area == 1||closeToWall == 1) {
					travelTo(TUNNEL_LL[0], TUNNEL_LL[1] - 1);
					reLocalize(TUNNEL_LL[0], TUNNEL_LL[1] - 1);
				} else if(closeToWater_area == 2||closeToWall == 2) {
					travelTo(TUNNEL_LL[0] - 1, TUNNEL_LL[1]);
					reLocalize(TUNNEL_LL[0] - 1, TUNNEL_LL[1]);
				} else if(closeToWater_area == 3||closeToWall == 3) {
					travelTo(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
					reLocalize(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
				} else {
					travelTo(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
					reLocalize(TUNNEL_LL[0] + 1, TUNNEL_LL[1] - 1);
				}
			}
		}
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
	}
	
	/**
	 * This method drives the robot to a given coordinates in the unit of a tile size.
	 * @param coords
	 * This input is a 2-element array for the coodinates in the unit of a tile size.
	 */
	public void travelTo(int[] coords) {
	    travelTo(coords[0], coords[1]);
    }
	
	public void travelTo(double[] coords) {
	    travelTo(coords[0], coords[1]);
	}
	
	/**
	 * This method will try to search for the closest can first, then capture the can and check the color. If the color is correct, then the robot will go back to base. Else, the robot will discard the can and look for the next one.
	 */
	public boolean search() {
		double[] nextPos = new double[] {Odometer.getX()+1, Odometer.getY()+1};
		boolean betterThisTime = false;
		setSpeed(100);
		double can_t = 0;
		double can_dis = TILE_SIZE*1.5;
		LEFT_MOTOR.rotate(convertAngle(90), true);
		RIGHT_MOTOR.rotate(-convertAngle(90), true);
		
		while(LEFT_MOTOR.isMoving()&&RIGHT_MOTOR.isMoving()) {
			double current_dis = US.getData();
			double current_t = Odometer.getT();
			
			if(current_dis<can_dis) {
				betterThisTime = true;
				can_dis = current_dis;
				can_t = current_t;
			}
				
		}
		
		if(betterThisTime) {
			turnTo(can_t);
			moveForward(can_dis);
			MainProgram.CLAW.close();
			//Identify color
			MainProgram.ID.identifyCan();
			travelTo(SEARCH_LL);
		} else {
			if(isInTheRange(SEARCH_LL, SEARCH_UR, nextPos)) {
				travelTo(nextPos);
				return search();
			}
			travelTo(SEARCH_LL);
		}
		
		//Next round
		return betterThisTime;
	}
	
	public static boolean weight() {
		LEFT_MOTOR.setAcceleration(500);
		RIGHT_MOTOR.setAcceleration(500);
		LEFT_MOTOR.setSpeed(100);
		RIGHT_MOTOR.setSpeed(100);
		long start = System.currentTimeMillis();
		LEFT_MOTOR.rotate(-convertDistance(5), true);
		RIGHT_MOTOR.rotate(-convertDistance(5), false);
		long end = System.currentTimeMillis();
		return end - start > HEAVY;
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
		RIGHT_MOTOR.setSpeed((int)(SPEED*FACTOR));
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
	public void reLocalize(double x, double y) {
	    x *= TILE_SIZE;
	    y *= TILE_SIZE;
	    int speed = SPEED;
	    setSpeed(200);
	    turnTo(0);
	    lightLoc();
		turnRight(90);
		lightLoc();
		turnLeft(90);
		setSpeed(speed);
		Odometer.setX(x);
		Odometer.setY(y);
		Odometer.resetTheta();
	}
	
	/**
	 * 
	 * This method localizes the robot at the starting corner.
	 * 
	 */
	public void localize() {
		ultrasonicLoc();
		int speed = SPEED;
		setSpeed(200);
		lightLoc();
		turnRight(90);
		lightLoc();
		turnLeft(90);
		setSpeed(speed);
		
		switch(START_CORNER) {
		case 0:
		    //Lower left
		    Odometer.setX(TILE_SIZE);
		    Odometer.setY(TILE_SIZE);
		    Odometer.setT(0);
		    position = new double[] {1,1,0};
		    break;
		case 1:
		    //Lower right
		    Odometer.setX(14*TILE_SIZE);
		    Odometer.setY(TILE_SIZE);
		    Odometer.setT(270);
		    position = new double[] {14,1,270};
		    break;
		case 2:
		    //Upper right
		    Odometer.setX(14*TILE_SIZE);
		    Odometer.setY(8*TILE_SIZE);
		    Odometer.setT(180);
		    position = new double[] {14,8,180};
		    break;
		case 3:
		    //Upper left
		    Odometer.setX(1*TILE_SIZE);
		    Odometer.setY(8*TILE_SIZE);
		    Odometer.setT(90);
		    position = new double[] {1,8,90};
		    break;
		default:
		    System.err.println("Error: Program failure at localization. Invalid starting corner");
		    Sound.beep();
            Sound.beep();
            Sound.beep();
            System.exit(0);
		}
		
		Sound.beep();
		Sound.beep();
		Sound.beep();
		
		if(DEBUG) {
            System.out.println(Odometer.getX());
            System.out.println(Odometer.getY());
            System.out.println(Odometer.getT());
            for(int i = 0; i < 9; i++) {
                System.out.println();
            }
            if(Button.waitForAnyPress() == Button.ID_ESCAPE) System.exit(0);
        }
	}

    private void lightLoc() {
        double dTheta, diff;
        boolean left = false, right = false;
        double leftDetection = 0, rightDetection = 0;
        
        //int speed = SPEED;
        //setSpeed(200);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.forward();
        do {
        	if(detectLineLeft()&&!left) {
            	left = true;
            	leftDetection = LEFT_MOTOR.getTachoCount();
            	if(right) {
            		stop();
            	}
            }
            if(detectLineRight()&&!right) {
            	right = true;
            	rightDetection = LEFT_MOTOR.getTachoCount();
            	if(left) {
            		stop();
            	}
            }
        } while(!left||!right);
        //setSpeed(speed);
        
        diff = 2 * Math.PI * RADIUS * (rightDetection - leftDetection) / 360;
        dTheta = Math.toDegrees(Math.atan(diff/LS_DIFF));
        
        turnLeft(dTheta);
        
        LEFT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), true);
        RIGHT_MOTOR.rotate(-convertDistance(Math.abs(diff)*Math.cos(Math.toRadians(dTheta)) + LS_TK_DIS), false);
        try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
    }

    private void ultrasonicLoc() {
        double[] pos1 = {0, 0, 0};
        double[] pos2 = {0, 0, 0};
        float d1, d2;
        double dTheta;
        boolean isDetected = false;
        try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
        
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.backward();
        
        try {Thread.sleep(TIME_INTERVAL*10);} catch (InterruptedException e) {}
        
        d1 = US.getData();
        
        // Rotate clockwise until you detect a falling edge
        while(!isDetected) {
        	try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
            LEFT_MOTOR.forward();
            RIGHT_MOTOR.backward();
            d2 = d1;
            d1 = US.getData(1);
            if((d2 >= (EDGE_DISTANCE + NOTICE_MARGIN))&&(d1<= (EDGE_DISTANCE - NOTICE_MARGIN))) {
            	stop();
            	isDetected = true;
            	pos1 = Odometer.getPosition();
            }
        }
        
        isDetected = false;
        
        LEFT_MOTOR.backward();
        RIGHT_MOTOR.forward();
        
        try {Thread.sleep(TIME_INTERVAL*10);} catch (InterruptedException e) {}
        
        d1 = US.getData();
        d2 = 0;
        
        // Rotate counter-clockwise until you detect another falling edge
        while(!isDetected) {
            try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e) {}
            LEFT_MOTOR.backward();
            RIGHT_MOTOR.forward();
            d2 = d1;
            d1 = US.getData(1);
            if((d2 >= (EDGE_DISTANCE + NOTICE_MARGIN))&&(d1<= (EDGE_DISTANCE - NOTICE_MARGIN))) {
            	stop();
            	isDetected = true;
            	pos2 = Odometer.getPosition();
            }
        }
        
        stop();
        
        // Calculate angle of local maximum based on two detected edges and use it to find 0° 
        
        if(pos1[2]<pos2[2]) {
        	//to air
        	dTheta = (135 + (pos1[2] + pos2[2])/2 + 360)%360;
        } else {
        	//to wall
        	dTheta = (-45+ 90 + (pos1[2] + pos2[2])/2 + 360)%360;
        }
        
        try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
        
        //Turn to 0
        turnTo(dTheta);
        turnRight(ROTATION_ERROR);
        Odometer.resetTheta();//Reset theta
        if(DEBUG) {
            System.out.println(Odometer.getX());
            System.out.println(Odometer.getY());
            System.out.println(Odometer.getT());
            for(int i = 0; i < 9; i++) {
                System.out.println();
            }
            if(Button.waitForAnyPress() == Button.ID_ESCAPE) System.exit(0);
        }
        
        try {Thread.sleep(TIME_INTERVAL);} catch (InterruptedException e1) {}
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
		RIGHT_MOTOR.rotate(-convertDistance(distance*FACTOR), false);
			
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
		RIGHT_MOTOR.rotate(convertDistance(distance*FACTOR), false);
		
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
	
	private static int convertAngle(double angle) {return convertDistance(Math.PI * TRACK * angle / 360.0);}
	  
	private static int convertDistance(double distance) {return (int) ((180.0 * distance) / (Math.PI * RADIUS));}
}