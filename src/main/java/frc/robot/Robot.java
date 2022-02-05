package frc.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpiutil.net.PortForwarder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
// import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Drive.*;
import frc.robot.Mechanisms.*;
import frc.robot.Autonomous.*;
// import frc.robot.Mechanisms.Flywheel;

public class Robot extends TimedRobot {
  private Auto Auto;

  private TorBalls torBalls;
  private VictorSPX hopperMainMotor;
  private CANSparkMax hopperShooterMotor;
  private VictorSPX intakeMotor;
  private Solenoid intakePiston;
  private final Compressor compressor;
  private TalonSRX climbTalon1;
  private VictorSPX colorwheelTalon;
  private Solenoid adjustingPiston;
  private Solenoid colorwheelPiston;
  private Schwingster climber;
  
  // private Drive drivecontrol;
  // private ColorWheel colorwheel;
  
  // private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private DriveHardware hardware;
	private TorDrive drive;
  private Joystick player1;
  private Joystick player2;
  private Joystick autoBox;

  public boolean test;
  // private String gameData;

  private double autoNumber;
  
  public Robot() {
    compressor = new Compressor();
    adjustingPiston = new Solenoid(4);
    // colorwheelPiston = new Solenoid(3);

    climbTalon1 = new TalonSRX(10);
    // colorwheelTalon = new VictorSPX(13);
  
      
    hardware = new DriveHardware();																																																																																																					
    player1 = new Joystick(0);
    player2 = new Joystick(1);
    autoBox = new Joystick(2);
		drive = new TorDrive(hardware, player1);

    test = false;
    
    hopperMainMotor = new VictorSPX(12);
    hopperShooterMotor = new CANSparkMax(9, MotorType.kBrushless);
    intakeMotor = new VictorSPX(3);
    intakePiston = new Solenoid(2);

    torBalls = new TorBalls(player2, hopperMainMotor, hopperShooterMotor, intakeMotor, intakePiston);
    climber = new Schwingster(player2, climbTalon1, adjustingPiston);
    // // colorwheel = new ColorWheel(colorwheelTalon, m_colorSensor, player2, colorwheelPiston);

    Auto = new Auto(torBalls, drive, player1);
  }
  
  @Override
  public void robotInit() { 
    compressor.start();
    hardware.init();
    
    //camera code
    // UsbCamera intakeCam = CameraServer.getInstance().startAutomaticCapture(0);
    // UsbCamera frontCam = CameraServer.getInstance().startAutomaticCapture(1);
    // intakeCam.setBrightness(50);
    // intakeCam.setFPS(30);
    // frontCam.setBrightness(50);
    // frontCam.setFPS(30);
		// CvSink cvsink1 = new CvSink("Intake and hopper Cam");
		// cvsink1.setSource(intakeCam);
    // cvsink1.setEnabled(false);
    // CvSink cvsink2 = new CvSink("Front Camera");
		// cvsink2.setSource(frontCam);
    // cvsink2.setEnabled(true);
    // CameraServer.getInstance().startAutomaticCapture();

    // PortForwarder.add(5800, "limelight.local", 5800);
    // PortForwarder.add(5801, "limelight.local", 5801);
    // PortForwarder.add(5805, "limelight.local", 5805);
  }
  
  @Override
  public void autonomousInit() {
    for(int i = 1; i<5; i++) {
      if(autoBox.getRawButton(i)) {
        autoNumber += Math.pow(2, i-1);
      }
    }
    Auto.setNumber(autoNumber);
  }

  @Override
  public void teleopInit() {
    //1 = blue, 2 = green, 3 = red, 4 = yellow
    // colorWheelRun();
   
    // torBalls.init();
    super.teleopInit();
  }

  @Override
  public void autonomousPeriodic() { 
    // drive.Run(test, true);
    // Auto.testRun();
    Auto.run();
  }

  @Override
  public void teleopPeriodic() {

    climber.run();
    // Auto.testRun();
    drive.Run(test, true);
    // colorWheelRun();
    torBalls.run(true, true);
    // colorwheel.Main();
    compressor.start();
    
    // SmartDashboard.putNumber("current position:", drive.getPosition());
    // SmartDashboard.putNumber("left encoder:", drive.getLeftEncoder());
    // SmartDashboard.putNumber("right encoder:", drive.getRightEncoder());
    // SmartDashboard.putNumber("current heading degrees:", (drive.getHeading() * 180 / Math.PI));
  }
  
  @Override
  public void testPeriodic() {
    //Auto.testRun(); 
    // drive.Run(test, true);
    System.out.println("Right: " + hardware.getRightEncoder() + "  Left: " + hardware.getLeftEncoder());
  }

  //Fetch Buttons
  public boolean getButtonA(){ return player1.getRawButton(1); }
	public boolean getButtonB(){ return player1.getRawButton(2); }
	public boolean getButtonX(){ return player1.getRawButton(3); }
	public boolean getButtonY(){ return player1.getRawButton(4); }
}