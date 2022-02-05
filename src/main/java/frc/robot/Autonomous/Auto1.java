package frc.robot.Autonomous;
import frc.robot.Mechanisms.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Autonomous.curveTrajectory.Direction;
import frc.robot.Drive.*;

public class Auto1 {
    public static enum autoRun {
        INIT, Curve1, //Linear1, Pivot1, //limeLightLineUp,
        //Shoot, 
        //Linear2, 
        DONE;
        private autoRun() {}
    }
    
    private TorBalls torBalls;
    private linearTrajectory linear1;
    //private linearTrajectory linear2;
    private pivotTrajectory pivot1;
    private curveTrajectory curve1;
    //private limelightLineUp limeLight1;
    private double currentTime;
    private double startTime;

    private autoRun autoState = autoRun.INIT;

    public Auto1(TorBalls torBalls, TorDrive torDrive) {
        //this.torBalls = torBalls;
        //linear1 = new linearTrajectory(torDrive, 8, 2.0);                   
        //pivot1 = new pivotTrajectory(torDrive, 60, 7.0);
        curve1 = new curveTrajectory(torDrive, 8, 3.0, Direction.LEFT);         
        //linear2 = new linearTrajectory(torDrive, 2, 5.0);
        //limeLight1 = new limelightLineUp(torDrive, 0.25, 1.0);
    }

    public void run() {
        currentTime = Timer.getFPGATimestamp();
        switch(autoState) {
            case INIT:
                curve1.init();
                System.out.println("start");
                //torBalls.autoRun(0);              
                autoState = autoRun.Curve1;
                break;
            case Curve1:
                curve1.run();
                //ystem.out.println("curve");
                //torBalls.autoRun(0);
                if(curve1.isDone()) {
                    System.out.println("curvedone");
                    //pivot1.init();
                    autoState = autoRun.DONE;
                }
                break;
            /*case Pivot1:
                pivot1.run();
                //System.out.println("pivot");
                //torBalls.autoRun(1);
                if(pivot1.isDone()) {                   
                    //limeLight1.init();
                    //linear2.init();
                    //System.out.println("pivotdone");
                    autoState = autoRun.DONE;
                }
                break;
                /*
            case limeLightLineUp:
                limeLight1.run();
                torBalls.autoRun(1);
                if(limeLight1.isDone()) {
                    startTime = currentTime;
                    autoState = autoRun.Shoot;
                }
                break;
            case Shoot:
                torBalls.autoRun(2);
                if(currentTime > startTime + 4.0) {
                    autoState = autoRun.DONE;
                }
                break;
                */
            /*
            case Linear2:
                linear2.run();
                if(pivot1.isDone()) {                   
                    //limeLight1.init();
                    //System.out.println("pivotdone");
                    autoState = autoRun.DONE;
                }
                break;
            */
            case DONE:
                //torBalls.autoRun(0);
                break;
        }
    }
}
