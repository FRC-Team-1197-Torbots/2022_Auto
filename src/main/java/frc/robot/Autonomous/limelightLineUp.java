package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drive.*;
import frc.robot.PID_Tools.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limelightLineUp {
	private TorDrive drive;
	private double speed;
	private boolean isFinished = false;
	private double lasttime;
	private double currentTime;
	private double minimalTime;
	private final double kF = 0.005;

	private final double maxSpeed = 0.75;
    
    //PID For rotation
	private final double rkP = -1.6;//1
	private final double rkD = 0.0;//-0.009
	private final double rkI = 0.05;//0.02
	
	//tolerances
	private final double angleTolerance = 1.0 * (Math.PI / 180.0);//radians
	private final double omegaTolerance = 1.5 * (Math.PI / 180.0);//radians per second
	
	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	private double omegaI;
	private final double lor = -1;
	
	private double angleError;
	
	private double currentVelocity;
	
	private double timeOutTime;
	
    private TorDerivative derivative;
    
    
    // for the limelight
    private NetworkTable table;
    private NetworkTableEntry tx;
	
	public static enum run {
		IDLE, GO;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public limelightLineUp(TorDrive drive, double minimalTime, double timeOutTime) {
		this.drive = drive;
		this.minimalTime = minimalTime;
		this.timeOutTime = timeOutTime;
        derivative = new TorDerivative(kF);
         
        table = NetworkTableInstance.getDefault().getTable("limelight");//bottom
        tx = table.getEntry("tx");
	}

	public boolean isDone() {
		return isFinished;
	}
	
	public void init() {
		isFinished = false;
		runIt = run.GO;
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        angleError = tx.getDouble(0.0) * Math.PI / 180.0;
        
		//we need to make sure control system is efficient so the angle error ranges from -pi to pi
		if(angleError > Math.PI) {
			angleError -= (2 * Math.PI);
		} else {
			if(angleError < -Math.PI) {
				angleError += (2 * Math.PI);
			}
		}
		derivative.resetValue(angleError);
		lasttime = Timer.getFPGATimestamp();
	}
	
	public void run() {
		currentTime = Timer.getFPGATimestamp();
		switch(runIt) {
		case IDLE:
			break;
		case GO:
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
            angleError = (tx.getDouble(0.0) - 1.75) * Math.PI / 180.0;
            
			//we need to make sure control system is efficient so the angle error ranges from -pi to pi
			if(angleError > Math.PI) {
				angleError -= (2 * Math.PI);
			} else {
				if(angleError < -Math.PI) {
					angleError += (2 * Math.PI);
				}
			}
			
			omegaI += angleError;
			omegaP = angleError * rkP;
			if(Math.abs(angleError) < angleTolerance) {
				omegaI = 0;
			}
			if(omegaI > (0.7 / (rkI * kF))) {
				omegaI = (0.7 / (rkI * kF));
			}
			if(omegaI < -(0.7 / (rkI * kF))) {
				omegaI = -(0.7 / (rkI * kF));
			}
			if(omegaP > 0.7) {
				omegaP = 0.7;
			}
			if(omegaD < -0.7) {
				omegaD = -0.7;
			}
			currentVelocity = derivative.estimate(angleError);//radians per second
			omegaD = (currentVelocity * rkD);
			
			speed = omegaP + omegaD + (omegaI * rkI * kF);
			speed *= lor;

			if(speed > maxSpeed) {
				speed = maxSpeed;
			} else if(speed < -maxSpeed) {
				speed = -maxSpeed;
			}
			
			drive.setMotorSpeeds(speed, -speed);
				
			if((((Math.abs(angleError) <= angleTolerance)
					&& (Math.abs(currentVelocity) < omegaTolerance))
					
					|| 
					(currentTime - lasttime > timeOutTime))//timeout
					&& (currentTime - lasttime > minimalTime)) {
				drive.setMotorSpeeds(0, 0);
				isFinished = true;
				NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
				runIt = run.IDLE;
			}
			break;
		}
	}
}