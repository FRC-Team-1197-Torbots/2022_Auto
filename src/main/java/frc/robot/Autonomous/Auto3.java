package frc.robot.Autonomous;
import frc.robot.Mechanisms.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drive.*;

public class Auto3{
    public static enum autoRun {
        INIT, limeLightLineUp,
        Shoot, Linear1, DONE;
        private autoRun() {}
    }
    
    private TorBalls torBalls;
    private linearTrajectory linear1;
    // private pivotTrajectory pivot1;
    private limelightLineUp limeLight1;
    private double currentTime;
    private double startTime;

    private autoRun autoState = autoRun.INIT;

    public Auto3(TorBalls torBalls, TorDrive torDrive) {
        this.torBalls = torBalls;
        linear1 = new linearTrajectory(torDrive, 5, 2);
        // pivot1 = new pivotTrajectory(torDrive, -20, 2.0);
        limeLight1 = new limelightLineUp(torDrive, 0.25, 1.0);
    }

    public void run() {
        currentTime = Timer.getFPGATimestamp();
        switch(autoState) {
            case INIT:

                limeLight1.init();
                torBalls.autoRun(0);
                autoState = autoRun.limeLightLineUp;
                break;
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
                if(currentTime > startTime + 5.0) {
                    linear1.init();
                    autoState = autoRun.Linear1;
                }
                break;
            case Linear1:
                linear1.run();
                torBalls.autoRun(0);
                autoState = autoRun.DONE;           
            
            case DONE:
                torBalls.autoRun(0);
                break;
        }
    }
}
