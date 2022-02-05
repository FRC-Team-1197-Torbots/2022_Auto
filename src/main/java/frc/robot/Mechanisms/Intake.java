package frc.robot.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;

public class Intake {
    private VictorSPX intakeMotor;
    private Solenoid intakePiston;
    public Intake(VictorSPX intakeMotor, Solenoid intakePiston) {
        this.intakeMotor = intakeMotor;
        this.intakePiston = intakePiston;
    }

    public void runState(int state) {
        //0 is idle
        //1 is intaking
        //2 is intake motor on but retracted (for shooting)
        if(state == 1) {
            intakeMotor.set(ControlMode.PercentOutput, 1);
            intakePiston.set(true);
        } else if(state == 2) {
            intakeMotor.set(ControlMode.PercentOutput, 0.25);
            intakePiston.set(false);
        } else if(state == 3) {
            intakeMotor.set(ControlMode.PercentOutput, 0);
            intakePiston.set(true);

        } else if(state == 4) {
            intakeMotor.set(ControlMode.PercentOutput, 0.75);
            intakePiston.set(true);
        } else {
            intakeMotor.set(ControlMode.PercentOutput, 0);
            intakePiston.set(false);
        }
    }
}
