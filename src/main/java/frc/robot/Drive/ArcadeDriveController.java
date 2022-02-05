package frc.robot.Drive;

import frc.robot.PID_Tools.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArcadeDriveController extends DriveController {

//    private DigitalInput colorWheelDIO;
   private double throttleAxis;
   private double arcadeSteerAxis;
   private double leftOutput;
   private double rightOutput;
   private double rightMotorSpeed;
   private double leftMotorSpeed;
   private Joystick player1;

   // for the limelight
   private NetworkTable table;
   private NetworkTableEntry tx;
   private NetworkTableEntry ta;
   private double x;
   private double speedChange;
   private double area;

   private double distance;
   private BantorPID limeLightPID;
   private double currentVelocity;
   private TorDerivative findCurrentVelocity;

   //time stuff to make sure only goes in correct intervals
   private long currentTime;
   private long startTime = (long) (Timer.getFPGATimestamp() * 1000);
   private double timeInterval = 0.005;
   private double dt = timeInterval;
   private long lastCountedTime;
   private boolean starting = true;

   /*
    * tuneable
    * things---------------------------------------------------------------->>>>>>>
    * >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    */
   private final double areaAt1Meter = 1.27;//in percent//maybe later?

   // limelight PID
   // Stuff------------------------------>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

   private final double positionkP = -2; //-1.4
   private final double positionkI = 0; //-0.002
   private final double positionkD = -0.035; //-0.035
   private final double positionTolerance = 3 * Math.PI / 180.0;// for thePID
   private final double velocitykP = 0.0;// velocity stuff probably not needed at all and should keep 0
   private final double velocitykI = 0.0;
   private final double velocitykD = 0.0;
   private final double kV = 0.0;// this should definitely stay at 0
   private final double kA = 0.0;// this should definitely stay at 0
   private final double velocityTolerance = 0.0;
   private final double targetVelocity = 0.0;// probably won't need
   private final double targetAcceleration = 0.0;// probably won't need

   // ------------------------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

   //this is for the curve drive

   //tunable
   private final double matrixLength = 2;
   private final double AmatrixLength = 1;

   //not tunable
   private final double weight = 1 / (matrixLength + 1);
   private final double Aweight = 1 / (AmatrixLength + 1);
   private double[] throttleArray = new double[(int)matrixLength];
   private double[] arcadeArray = new double[(int)AmatrixLength];
   private boolean currentInit = true;

   /*
    * no more tuneable
    * things-------------------------------------------------------->>>>>>>>>>>>>>>
    * >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    */

   public ArcadeDriveController(DriveHardware hardware, Joystick player1) {
       super(hardware, player1);
       this.player1 = player1;
    //    colorWheelDIO = new DigitalInput(0);

       // this is the PID
       limeLightPID = new BantorPID(kV, kA, positionkP, positionkI, positionkD, velocitykP, velocitykI, velocitykD, dt,
               positionTolerance, velocityTolerance);
       limeLightPID.reset();
       findCurrentVelocity = new TorDerivative(dt);
       findCurrentVelocity.resetValue(0);
       table = NetworkTableInstance.getDefault().getTable("limelight");//bottom
       tx = table.getEntry("tx");
       ta = table.getEntry("ta");
   }

   @Override
   public void init() {
       table = NetworkTableInstance.getDefault().getTable("limelight");//bottom
       tx = table.getEntry("tx");
       ta = table.getEntry("ta");
   }

   @Override
   public void run() {
       if(currentInit) {
           for(double lambda:throttleArray)
               lambda = 0;
           for(double lambda:arcadeArray)
               lambda = 0;
           currentInit = false;
       }
       currentTime = (long) (Timer.getFPGATimestamp() * 1000);
       // this handles it so that it will only tick in the time interval so that the derivatives and the integrals are correct
       if (((currentTime - startTime) - ((currentTime - startTime) % (dt * 1000))) > // has current time minus start time to see the relative time the trajectory has been going
       ((lastCountedTime - startTime) - ((lastCountedTime - startTime) % (dt * 1000))) // subtracts that mod dt times 1000 so that it is floored to
       // the nearest multiple of dt times 1000 then checks if that is greater than the last one to see if it is time to move on to the next tick
       || starting) {
           starting = false;
           lastCountedTime = currentTime;
          
           throttleAxis = -player1.getRawAxis(1);
           arcadeSteerAxis = player1.getRawAxis(0);
           if (Math.abs(arcadeSteerAxis) <= 0.15) {
               arcadeSteerAxis = 0.0;
           }
           if (Math.abs(throttleAxis) <= 0.15) {
               throttleAxis = 0.0;
           }
           arcadeSteerAxis = Math.pow(arcadeSteerAxis, 3);
           throttleAxis = Math.pow(throttleAxis, 3);

           if(player1.getRawButton(2)) {
               throttleAxis *= 0.25;
               arcadeSteerAxis *= 0.25;
           } else if(player1.getRawButton(3)) {
            throttleAxis *= 0.125;
            arcadeSteerAxis *= 0.125;
           }
           // get all the values from the limelight
        //    SmartDashboard.putNumber("tx:", tx.getDouble(0.0) - 1.5);
           x = tx.getDouble(0.0) - 1.5;
        //    area = ta.getDouble(0.0);

           // convert tShe angles into radians
           x *= ((Math.PI) / 180.0);
        //    SmartDashboard.putNumber("x:", x);
        //    distance = areaAt1Meter / area;

        //    SmartDashboard.putNumber("distance limelight", distance);
           if(player1.getRawButton(1)) {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);//light up
                currentVelocity = findCurrentVelocity.estimate(x);

                limeLightPID.updateTargets(0 * (Math.PI / 180.0), targetVelocity, targetAcceleration);
                limeLightPID.updateCurrentValues(x, currentVelocity);
                speedChange = limeLightPID.update();
                // SmartDashboard.putNumber("speedChanged right now:", speedChange);
                arcadeSteerAxis += speedChange;
           } else {
               NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
           }

           throttleAxis *= weight;
           for(double lambda:throttleArray) {
               throttleAxis += weight * lambda;
           }
           for(int i = ((int)matrixLength - 1); i >= 1; i--) {
               throttleArray[i] = throttleArray[i - 1];
           }
           throttleArray[0] = throttleAxis;

           arcadeSteerAxis *= Aweight;
           for(double lambda:arcadeArray) {
               arcadeSteerAxis += Aweight * lambda;
           }
           for(int c = ((int)AmatrixLength - 1); c >= 1; c--) {
               arcadeArray[c] = arcadeArray[c - 1];
           }
           arcadeArray[0] = arcadeSteerAxis;
           arcadeSteerAxis *= 0.5;//we have to slow down

           if (throttleAxis > 0.0D) {
               if (arcadeSteerAxis > 0.0D) {
                   leftMotorSpeed = throttleAxis - arcadeSteerAxis;
                   rightMotorSpeed = Math.max(throttleAxis, arcadeSteerAxis);
               } else {
                   leftMotorSpeed = Math.max(throttleAxis, -arcadeSteerAxis);
                   rightMotorSpeed = throttleAxis + arcadeSteerAxis;
               }
           } else {
               if (arcadeSteerAxis > 0.0D) {
                   leftMotorSpeed = -Math.max(-throttleAxis, arcadeSteerAxis);
                   rightMotorSpeed = throttleAxis + arcadeSteerAxis;
               } else {
                   leftMotorSpeed = throttleAxis - arcadeSteerAxis;
                   rightMotorSpeed = -Math.max(-throttleAxis, -arcadeSteerAxis);
               }
           }           

           //for color wheel DIO
        //    if(!player1.getRawButton(3)) {
        //        rightMotorSpeed = 0;
        //        leftMotorSpeed = 0;
        //    }
           
           setRightOutput(rightMotorSpeed);
           setLeftOutput(leftMotorSpeed);
       }
   }

   @Override
   public double getLeftOutput() {
       return leftOutput;
   }

   @Override
   public void setLeftOutput(double left) {
       leftOutput = left;
   }

   @Override
   public double getRightOutput() {
       return rightOutput;
   }

   @Override
   public void setRightOutput(double right) {
       rightOutput = right;
   }

   @Override
   public void limeLightTop(boolean top) {
    //    limeLightTop = top;
   }

   public void setTargets(double velocity, double omega) {
       //sets leftMotorSpeed and rightMotorSpeed

   }

   public double function(double x) {
       if(x >= 0) {
           return (0.5) * (1 - Math.cos(Math.PI * (Math.pow(x, 1.75))));
       } else {
           return -(0.5) * (1 - Math.cos(Math.PI * (Math.pow(Math.abs(x), 1.75))));
       }
   }
}
