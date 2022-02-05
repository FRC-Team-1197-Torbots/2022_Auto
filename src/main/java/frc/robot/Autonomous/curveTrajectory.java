package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.PID_Tools.*;
import frc.robot.Drive.*;

public class curveTrajectory {
    private TorDrive torDrive;
    //private boolean direction;
    private double timeout;
    private double outer_distance;
    //private double outer_velocity;
    //private double inner_velocity;
    private boolean isFinished = false;

    private final double maxSpeed = 0.7;

    private double velocity;
    private double currentVelocity;
    private double startDistance;
    private double currentDistance;
    private double currentTime;
    private double lastTime;
    private double error;
    private TorDerivative derivative;

    private final double kF = 0.005;
    private final double errorFob = 1;
    private final double positionTolerance = 0.05;//units: feet 0.015
    private final double velocityTolerance = 0.1;//units: feet per second 0.015
	//private final double headingTolerance = 1.5 * (Math.PI / 180.0);//units: radians 2.5 degrees

    private final double tkP = 0.2;//0.5
	private final double tkD = 0.0002;//0.00002;//.001
    private final double tkI = 0.05;//.0003//initial 0.05

    private double vP;//velocity proportional
	private double vD;//velocity derivative
	private double vI = 0;
    
    public static enum run{
        IDLE, GO;
        private run(){
        }
    }

    //Eventually, we'll replace the "direction" boolean with this enum
    public enum Direction {
        LEFT, RIGHT;
    }

    private Direction direction;

    public run runCurve = run.IDLE;

    //constructor that takes TorDrive, outter distance, timeout, LeftORRight turn
    public curveTrajectory(TorDrive torDrive, double outer_distance, double timeout, Direction direction){
        this.torDrive = torDrive;
        this.outer_distance = outer_distance;
        this.timeout = timeout;
		this.direction = direction;
		derivative = new TorDerivative(kF);
    }

    public boolean isDone() {
		return isFinished;
	}

    public void init(){
        isFinished = false;
        runCurve = run.GO;
        startDistance = torDrive.getPosition();
        //ANGLE STUFF!
        /*
		firstAngle = drive.getHeading();
		currentAngle = drive.getHeading();
		angleError = currentAngle - firstAngle;
		is in radians so we have to make sure that it goes from -pi to pi and does not have 
		an absolute value greater than pi in order to be an efficient control system
        
		if(angleError > Math.PI) {
			angleError -= (2 * Math.PI);
		} else {
			if(angleError < -Math.PI) {
				angleError += (2 * Math.PI);
			}
		}
		angleDerivative.resetValue(angleError);
        */
		derivative.resetValue(torDrive.getPosition());
		lastTime = Timer.getFPGATimestamp();
    }

    public void run() {

        //currentAngle = drive.getHeading();
		//we can't fix current angle right now so that it can't be 359 degrees since we need it in this raw value first for the angleError
		//since it is never used other than for finding angleError, there is no need to make sure that it reads -1 degrees rather than 359 degrees
		currentDistance = torDrive.getPosition();
		currentTime = Timer.getFPGATimestamp();
        switch(runCurve) {
		case IDLE:
			break;
		case GO:
            
            //ANGLE CORRECTION STUFF, DON'T USE!
            /*
			angleError = currentAngle - firstAngle;
			//is in radians so we have to make sure that it goes from -pi to pi and does not have 
			//an absolute value greater than pi in order to be an efficient control system
            
			if(angleError > Math.PI) {
				angleError -= (2 * Math.PI);
			} else {
				if(angleError < -Math.PI) {
					angleError += (2 * Math.PI);
				}
			}
            */
			
			//since this distance is always positive, we have to multiply by fob for if it is negative
			error = (outer_distance) - (currentDistance - startDistance);//error always positive if approaching
			error *= errorFob;
			vI += error;
			if(Math.abs(error) <= positionTolerance * 0.5) {
				vI = 0;
			}
			if(vI > (0.7 / (tkI * kF))) {
				vI = (0.7 / (tkI * kF));
			}
			if(vI < -(0.7 / (tkI * kF))) {
				vI = -(0.7 / (tkI * kF));
			}
			vP = error * tkP;
			
			currentVelocity = derivative.estimate(torDrive.getPosition());//almost always positive
			//has to be multiplied by -1 so that if it is approaching the target to fast
			//it does not act as a positive. Because, if it was approaching fast, the
			//derivative would be positive
			vD = (currentVelocity) * tkD * (180 / Math.PI);//degrees per second
			velocity = vP + vD + (vI * tkI * kF);
			//velocity is good
            
            if(velocity > maxSpeed) {
				velocity = maxSpeed;
			} else if(velocity < -maxSpeed) {
				velocity = -maxSpeed;
			}
			System.out.println("Velocity: "+velocity);
			torDrive.setMotorSpeeds(velocity * 0.7, velocity);//left, right
				if((Math.abs(error) <= positionTolerance
						//&& Math.abs(angleError) <= headingTolerance
						&& Math.abs(currentVelocity) < velocityTolerance)
						|| (currentTime - lastTime > timeout))
				{
					torDrive.setMotorSpeeds(0, 0);
					isFinished = true;
					runCurve = run.IDLE;
				}
				break;
			}
        }

    //Hard coded "turn factor" <<ratio of outside wheel drive to inner wheel drive
}
