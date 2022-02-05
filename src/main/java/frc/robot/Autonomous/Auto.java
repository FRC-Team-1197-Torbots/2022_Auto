package frc.robot.Autonomous;

import frc.robot.Mechanisms.*;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drive.*;

public class Auto {
    private double autoToRun = 0;
    private TorBalls torBalls;
    private TorDrive torDrive;
    private Joystick player1;

    private Auto1 Auto1;
    private Auto2 Auto2;
    //private Auto3 Auto3;
    //private Auto4 Auto4;

    private linearTrajectory linearRun;
    private pivotTrajectory pivotRun;
    
    private linearTrajectory linearRun2;
    private pivotTrajectory pivotRun2;
    private limelightLineUp limeLight;
    
    public static enum testAuto {   
        IDLE, LinearRun, PivotRun, LinearRun2, PivotRun2, limeLight;
        private testAuto() {}
    }
    
    private testAuto testAutoStateMachine = testAuto.IDLE;

    public Auto(TorBalls torBalls, TorDrive torDrive, Joystick player1) {
        this.torBalls = torBalls;
        this.player1 = player1;
        this.torDrive = torDrive;

        linearRun = new linearTrajectory(torDrive, 6.0, 100.0);//want to tune it with infinite time
        pivotRun = new pivotTrajectory(torDrive, 90.0, 100.0);
        
        linearRun2 = new linearTrajectory(torDrive, -3.0, 100.0);//want to tune it with infinite time
        pivotRun2 = new pivotTrajectory(torDrive, -180.0, 100.0);
       // limeLight = new limelightLineUp(torDrive, 0.5, 100.0);

        Auto1 = new Auto1(torBalls, torDrive);
        Auto2 = new Auto2(torBalls, torDrive);
        //Auto3 = new Auto3(torBalls, torDrive);
        //Auto4 = new Auto4(torBalls, torDrive);
    }

    public void testRun() {
        // SmartDashboard.putNumber("current position:", torDrive.getPosition());
        // SmartDashboard.putNumber("left encoder:", torDrive.getLeftEncoder());
        // SmartDashboard.putNumber("right encoder:", torDrive.getRightEncoder());
        // SmartDashboard.putNumber("current heading degrees:", (torDrive.getHeading() * 180 / Math.PI));

        // SmartDashboard.putString("state machine", testAutoStateMachine.toString());
        // SmartDashboard.putBoolean("linear is done", linearRun.isDone());
        // SmartDashboard.putBoolean("pivot is done", pivotRun.isDone());
        
        // SmartDashboard.putBoolean("linear 2 is done", linearRun2.isDone());
        // SmartDashboard.putBoolean("pivot 2 is done", pivotRun2.isDone());
        // SmartDashboard.putBoolean("limelight is done", limeLight.isDone());

        torBalls.autoRun(0);
        if(testAutoStateMachine == testAuto.IDLE) {
            if(player1.getRawButton(1)) {
                linearRun.init();
                testAutoStateMachine = testAuto.LinearRun;
            } else if(player1.getRawButton(2)) {
                pivotRun.init();
                testAutoStateMachine = testAuto.PivotRun;
            } else if(player1.getRawButton(3)) {
                linearRun2.init();
                testAutoStateMachine = testAuto.LinearRun2;

            } else if(player1.getRawButton(4)) {
                pivotRun2.init();
                testAutoStateMachine = testAuto.PivotRun2;
                
            } else if(player1.getRawButton(5)) {
                limeLight.init();
                testAutoStateMachine = testAuto.limeLight;
            }
        }
        switch(testAutoStateMachine) {
            case IDLE:
                break;
            case LinearRun:
                linearRun.run();
                if(linearRun.isDone()) {
                    testAutoStateMachine = testAuto.IDLE;
                }
                break;
            case PivotRun:
                pivotRun.run();
                if(pivotRun.isDone()) {
                    testAutoStateMachine = testAuto.IDLE;
                }
                break;
                case LinearRun2:
                    linearRun2.run();
                    if(linearRun2.isDone()) {
                        testAutoStateMachine = testAuto.IDLE;
                    }
                    break;
                case PivotRun2:
                    pivotRun2.run();
                    if(pivotRun2.isDone()) {
                        testAutoStateMachine = testAuto.IDLE;
                    }
                    break;
                case limeLight:
                    limeLight.run();
                    if(limeLight.isDone()) {
                        testAutoStateMachine = testAuto.IDLE;
                    }
                    break;
        }
    }

    public void setNumber(double x) {
        autoToRun = x;
    }

    public void run() {
        if(autoToRun == 1) {
            //System.out.println("Auto 1 is running");
            Auto1.run();
        } else if(autoToRun == 2) {
            System.out.println("Auto 2 is running");
            Auto2.run();
        } //else if(autoToRun == 3) {
            //Auto3.run();
        //} //else if(autoToRun == 4) {
            //Auto4.run();
        //}
    }
}
