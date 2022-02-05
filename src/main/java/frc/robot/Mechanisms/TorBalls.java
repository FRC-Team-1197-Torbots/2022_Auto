package frc.robot.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class TorBalls {
    private Flywheel flywheel;
    private Joystick player2;
    private VictorSPX intakeMotor;
    private Solenoid intakePiston;
    private Intake intake;
    private VictorSPX hopperMainMotor;
    private CANSparkMax hopperShooterMotor;

    public TorBalls(Joystick player2, VictorSPX hopperMainMotor, CANSparkMax hopperShooterMotor,
        VictorSPX intakeMotor, Solenoid intakePiston) {
        this.player2 = player2;
        this.hopperMainMotor = hopperMainMotor;
        this.hopperShooterMotor = hopperShooterMotor;
        this.intakeMotor = intakeMotor;
        this.intakePiston = intakePiston;
        intake = new Intake(this.intakeMotor, this.intakePiston);
        flywheel = new Flywheel(player2);
    }

    public void init() {
        flywheel.resetEncoder();
    }
    
    public void run(boolean flywheelRun, boolean hopperRun) {
        flywheel.run(flywheelRun, false);
        if(hopperRun) {
            if(player2.getRawButton(6)) {
                hopperShooterMotor.set(-0.95);
            } else {
                hopperShooterMotor.set(0.0);
            }
            if((Math.abs(player2.getRawAxis(3)) > 0.3)
            && flywheel.isFastEnough()) {//right trigger go
                intake.runState(2);
                hopperMainMotor.set(ControlMode.PercentOutput, -0.5);
            } else {
                if(Math.abs(player2.getRawAxis(2)) > 0.3) {
                    intake.runState(4);
                } else {
                    if(player2.getRawButton(7)) {
                        intake.runState(3);
                    } else {
                        intake.runState(0);
                    }
                }
                hopperMainMotor.set(ControlMode.PercentOutput, 0.0);
            }
        } else {
            hopperMainMotor.set(ControlMode.PercentOutput, 0.0);
            hopperShooterMotor.set(0.0);
        }
    }

    public void autoRun(int state) {
        /*we have different states
        0. Idle
        1. Shooter revving up
        2. Shooter firing
        3. intaking
        */

        if(state == 1) {//rev up
            //rev up shooter
            flywheel.run(true, true);
            intake.runState(0);
            hopperMainMotor.set(ControlMode.PercentOutput, 0.0);
            hopperShooterMotor.set(-0.95);
        } else if(state == 2) {//fire
            //rev up shooter
            flywheel.run(true, true);
            if(flywheel.isFastEnough()) {
                intake.runState(2);
                hopperMainMotor.set(ControlMode.PercentOutput, -1.0);
            }
            hopperShooterMotor.set(-0.95);
        } else if(state == 3) {//intake
            //shooter off
            flywheel.run(true, false);
            intake.runState(1);
            hopperMainMotor.set(ControlMode.PercentOutput, 0.0);
            hopperShooterMotor.set(0.0);
            
        } else if(state == 5) {//fire slowly
            //rev up shooter
            flywheel.run(true, true);
            if(flywheel.isFastEnough()) {
                intake.runState(2);
                hopperMainMotor.set(ControlMode.PercentOutput, -0.75);
            }
            hopperShooterMotor.set(-0.95);
        } else {//idle
            //shooter off
            flywheel.run(true, false);
            intake.runState(0);
            hopperMainMotor.set(ControlMode.PercentOutput, 0.0);
            hopperShooterMotor.set(0.0);
        }
    }
}
