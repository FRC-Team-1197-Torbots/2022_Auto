package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.PID_Tools.*;
import frc.robot.Drive.*;

public class linearTrajectory {
	private TorDrive drive;
	private double thisdistance;
	private double currentDistance;
	private boolean isFinished = false;
	
	private final double maxSpeed = 0.7; //original: 0.7
    
    //PID for translation
	private final double tkP = 0.2;//0.5 //original: 0.2
	private final double tkD = 0.00002;//.001
    private final double tkI = 0.05;//.0003
    
    //PID For rotation
	private final double rkP = -0.3;//0.5
	private final double rkD = 0.0;//0.0
	private final double rkI = 0.001;//0.01
	private final double kF = 0.005;
	private final int lor = 1;
	private final int errorFob = 1;//forwards or backwards
	
	//tolerances
	private final double positionTolerance = 0.05;//units: feet 0.015
	private final double velocityTolerance = 0.1;//units: feet per second 0.015
	private final double headingTolerance = 1.5 * (Math.PI / 180.0);//units: radians 2.5 degrees
	
	private double currentVelocity;
	
	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	private double omegaI = 0;
	
	private double vP;//velocity proportional
	private double vD;//velocity derivative
	private double vI = 0;
	
	private double omega;
	private double velocity;
	
	private double firstAngle;
	private double currentAngle;
	private double angleError;
	private double error;
	private double startDistance;
	private double lastTime;
	private double currentTime;
	private TorDerivative derivative;
	private TorDerivative angleDerivative;

	private double timeOutTime;
	
	
	public static enum run {
		IDLE, GO;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public linearTrajectory(TorDrive drive, double distance, double timeOutTime) {
		this.drive = drive;
		this.thisdistance = distance;
		this.timeOutTime = timeOutTime;
		derivative = new TorDerivative(kF);
		angleDerivative = new TorDerivative(kF);
	}
	
	public boolean isDone() {
		return isFinished;
	}
	public void init() {
		isFinished = false;
		runIt = run.GO;
		startDistance = drive.getPosition();
		firstAngle = drive.getHeading();
		currentAngle = drive.getHeading();
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
		angleDerivative.resetValue(angleError);
		derivative.resetValue(drive.getPosition());
		lastTime = Timer.getFPGATimestamp();
	}
		
	
	public void run() {
		currentAngle = drive.getHeading();
		//we can't fix current angle right now so that it can't be 359 degrees since we need it in this raw value first for the angleError
		//since it is never used other than for finding angleError, there is no need to make sure that it reads -1 degrees rather than 359 degrees
		currentDistance = drive.getPosition();
		currentTime = Timer.getFPGATimestamp();
		switch(runIt) {
		case IDLE:
			break;
		case GO:
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
			
			//since this distance is always positive, we have to multiply by fob for if it is negative
			error = (thisdistance) - (currentDistance - startDistance);//error always positive if approaching
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
			
			currentVelocity = derivative.estimate(drive.getPosition());//almost always positive
			//has to be multiplied by -1 so that if it is approaching the target to fast
			//it does not act as a positive. Because, if it was approaching fast, the
			//derivative would be positive
			vD = (currentVelocity) * tkD * (180 / Math.PI);//degrees per second
			velocity = vP + vD + (vI * tkI * kF);
			//System.out.println("velocity: " + velocity);
			//velocity is good
			omegaP = angleError * rkP;
			omegaI += angleError;
			if(Math.abs(angleError) < headingTolerance) {
				omegaI = 0;
			}
			if(omegaI > ((0.25) / (rkI * kF))) {
				omegaI = ((0.5) / (rkI * kF));
			}
			if(omegaI < -((0.25) / (rkI * kF))) {
				omegaI = -((0.5) / (rkI * kF));
			}
			
			omegaD = (angleDerivative.estimate(angleError)) * rkD;
			omega = omegaP + omegaD + (omegaI * rkI * kF);
			omega *= lor;

			if(velocity > maxSpeed) {
				velocity = maxSpeed;
				//System.out.println("velocity:" + velocity);
			} else if(velocity < -maxSpeed) {
				velocity = -maxSpeed;
				//System.out.println("velocity:" + velocity);
			}
			
			drive.setMotorSpeeds(velocity + omega, velocity - omega);//right, left	
				if((Math.abs(error) <= positionTolerance
						&& Math.abs(angleError) <= headingTolerance
						&& Math.abs(currentVelocity) < velocityTolerance)
						|| (currentTime - lastTime > timeOutTime))
				{
					drive.setMotorSpeeds(0, 0);
					isFinished = true;
					runIt = run.IDLE;
				}
				break;
			}
		}
	}