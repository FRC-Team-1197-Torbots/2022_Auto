package frc.robot.Autonomous;
import frc.robot.Mechanisms.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drive.*;

public class Auto4 {
    public static enum autoRun {
        INIT, Linear1, Pivot1, limeLightLineUp,
        Shoot, DONE;
        private autoRun() {}
    }
    
    private TorBalls torBalls;
    private linearTrajectory linear1;
    private pivotTrajectory pivot1;
    private limelightLineUp limeLight1;
    private double currentTime;
    private double startTime;

    private autoRun autoState = autoRun.INIT;

    public Auto4(TorBalls torBalls, TorDrive torDrive) {
        this.torBalls = torBalls;
        linear1 = new linearTrajectory(torDrive, -2.0, 5.0);
        pivot1 = new pivotTrajectory(torDrive, 0, 0);
        limeLight1 = new limelightLineUp(torDrive, 0.25, 2.5);
    }

    public void run() {
        currentTime = Timer.getFPGATimestamp();
        switch(autoState) {
            case INIT:
                linear1.init();
                torBalls.autoRun(0);
                autoState = autoRun.Linear1;
                break;
            case Linear1:
                linear1.run();
                torBalls.autoRun(0);
                if(linear1.isDone()) {
                    pivot1.init();
                    autoState = autoRun.Pivot1;
                }
                break;
            case Pivot1:
                pivot1.run();
                torBalls.autoRun(1);
                if(pivot1.isDone()) {
                    limeLight1.init();
                    autoState = autoRun.limeLightLineUp;
                }
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
                if(currentTime > startTime + 4.0) {
                    autoState = autoRun.DONE;
                }
                break;
            case DONE:
                torBalls.autoRun(0);
                break;
        }
    }
}
