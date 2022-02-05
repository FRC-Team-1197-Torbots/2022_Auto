package frc.robot.Drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;

public class DriveHardware {
	
	private ADXRS450_Gyro gyro;
	
	private final CANSparkMax rightMaster;
	private final CANSparkMax rightSlave1;
	private final CANSparkMax rightSlave2;
	private final CANSparkMax leftMaster;
	private final CANSparkMax leftSlave1;
	private final CANSparkMax leftSlave2;
	private final Encoder leftEncoder;
	private final Encoder rightEncoder;

	private double leftSpeed;
	private double rightSpeed;

	private double currentVoltage;

	private final Solenoid solenoid;

	/***************************/

	private final boolean leftOutputReversed = true;
	private final boolean rightOutputReversed = true;

	private double heading = 0.0;

	/*
	 * THINGS NEEDED TO BE TUNED --------------------------------------------
	 */
	// need to be in the same gear
	// also need to tune the values in TorTrajectory, to use linear and pivot
	// trajectories
	// also need to tune global motion limits in TorTrajectoryLib (don't need to
	// tune TorTrajectory in TorTrajectoryLib since it is not building any linear or
	// pivots)
	private final double encoderTicksPerFoot = 5460.0; // push the robot forward one foot and take the average of the
														// two encoder distances
	private final double absoluteMaxVelocity = 0.0; // use encoder ticks per foot, and using the robot, set it to the
													// max speed on both wheels and see how many encoder ticks it goes
													// forward
	// then using the encoder ticks per foot calculation, calculate its absolute Max
	// Velocity [Units: Feet/Second]
	private final double absoluteMaxAcceleration = 0.0;// [Units:(delta feet/seconds)/seconds
	// just see how many seconds it takes to go from 0 to 100% speed and divide the
	// absolute max velocity by that number of seconds
	private final double absoluteMaxOmega = 0.0;// use the gyro and set one motor to 100% and the other to -100% [Units:
												// Radians/Second]
	private final double absoluteMaxAlpha = 0.0;// [Units:(delta radians/seconds)/seconds]
	/*
	 * --------------------------------------------
	 */

	public DriveHardware() {
		gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

		solenoid = new Solenoid(0);

		leftMaster = new CANSparkMax(4, MotorType.kBrushless);
		leftSlave1 = new CANSparkMax(5, MotorType.kBrushless);
		leftSlave2 = new CANSparkMax(6, MotorType.kBrushless);
		rightMaster = new CANSparkMax(1, MotorType.kBrushless);
		rightSlave1 = new CANSparkMax(2, MotorType.kBrushless);
		rightSlave2 = new CANSparkMax(3, MotorType.kBrushless);

		leftEncoder = new Encoder(10, 11, false, Encoder.EncodingType.k4X);
		rightEncoder = new Encoder(12, 13, false, Encoder.EncodingType.k4X);

		leftMaster.setInverted(false); // Left master must be attached to the farthest CIM from the output shaft
		leftSlave1.setInverted(false);
		leftSlave2.setInverted(false);

		rightMaster.setInverted(false); // Right master must be attached to the farthest CIM from the output shaft
		rightSlave1.setInverted(false);
		rightSlave2.setInverted(false);

		resetEncoder();
		resetGyro();
	}

	public void setMotorSpeeds(final double leftSpeed, final double rightSpeed) {
		// SmartDashboard.putNumber("left raw", getLeftEncoder());
		// SmartDashboard.putNumber("right raw", getRightEncoder());
		SetLeft(leftSpeed);
		SetRight(-rightSpeed);
	}

	// Setting the left master Talon's speed to the given parameter
	public void SetLeft(final double speed) {
		// SmartDashboard.putNumber("set left:", speed);
		leftMaster.set(speed);
		leftSlave1.set(-speed);
		leftSlave2.set(-speed);
		// leftMaster.set(0.25);
		// leftSlave1.set(0.25);
		// leftSlave2.set(0.25);
	}

	// Setting the right master Talon's speed to the given parameter
	public void SetRight(final double speed) {
		// SmartDashboard.putNumber("set right:", speed);
		rightMaster.set(speed);
		rightSlave1.set(-speed);
		rightSlave2.set(-speed);
		// rightMaster.set(0.25);
		// rightSlave1.set(0.25);
		// rightSlave2.set(0.25);
	}

	// Getting raw position value from the right encoder
	public double getRightEncoder() {
		return rightEncoder.getRaw();
	}

	// Getting raw position value from the left encoder
	public double getLeftEncoder() {
		return leftEncoder.getRaw();
	}

	// Getting the average encoder position from both encoders
	public double getAverageEncoderPosition() {
		return (rightEncoder.getRaw() + leftEncoder.getRaw()) * 0.5;
	}

	// Getting the position from both encoders in feet
	public double getPosition() {
		final double left = leftEncoder.getRaw() / encoderTicksPerFoot;
		final double right = rightEncoder.getRaw() / encoderTicksPerFoot;

		//System.out.println("Right : " + right + "  ||  Left: " + left);
		return right; // [feet]
	}

	// Getting the angle in radians from the spartan board
	public double getHeading() {
		heading = (gyro.getAngle() * (Math.PI / 180));
		return heading; // [radians]
	}

	// Method to set the the linear and angular speed of the robot
	public void setTargets(final double percentV, final double percentOmega) {
		leftSpeed = percentV - percentOmega;
		rightSpeed = percentV + percentOmega;
		
		rightMaster.set(-rightSpeed);
		rightSlave1.set(rightSpeed);
		leftMaster.set(-leftSpeed);
		leftSlave1.set(leftSpeed);
	}

	// Method to reset the encoder values
	public void resetEncoder() {
		leftEncoder.reset();
		rightEncoder.reset();
	}

	// Method to reset the spartan board gyro values
	public void resetGyro() {
		gyro.reset(); 
	}
	
	// Method to shift the drive to low gear
	public void shiftToLowGear() {
		solenoid.set(true);
	}
	
	// Method to shift the drive to high gear
	public void shiftToHighGear() {
		solenoid.set(false);
	}
	
	// Method to initialize 
	public void init(){
		if(gyro == null){ 
			gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		}
		gyro.calibrate();
		// resetEncoder();
	}
	
	public double getAbsoluteMaxVelocity() {
		return absoluteMaxVelocity;
	}
	
	public double getAbsoluteMaxAcceleration() {
		return absoluteMaxAcceleration;
	}
	
	public double getAbsoluteMaxOmega() {
		return absoluteMaxOmega;
	}
	
	public double getAbsoluteMaxAlpha() {
		return absoluteMaxAlpha;
	}
	
	public double getCurrentVoltage() {
		currentVoltage = RobotController.getInputVoltage();
		return currentVoltage;
	}
}