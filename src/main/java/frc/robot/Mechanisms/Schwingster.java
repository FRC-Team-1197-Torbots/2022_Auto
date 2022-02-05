package frc.robot.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

public class Schwingster {
    private long currentTime = (long) (1000 * Timer.getFPGATimestamp());
    private long lastTimeButtonUpPressed = currentTime;
    private boolean startedUp = false;
    private Joystick player2;
    private TalonSRX climbTalon1;
    private Solenoid adjustingPiston;
    public Schwingster(Joystick player2, TalonSRX climbTalon1, Solenoid adjustingPiston) {
        this.player2 = player2;
        this.climbTalon1 = climbTalon1;
        this.adjustingPiston = adjustingPiston;
    }

    public void run() {
        currentTime = (long) (1000 * Timer.getFPGATimestamp());
        if(player2.getRawButton(4)) {
            climbTalon1.set(ControlMode.PercentOutput, 0.8);
        } else if(player2.getRawButton(3)) {
            climbTalon1.set(ControlMode.PercentOutput, -0.9);
        } else {
            climbTalon1.set(ControlMode.PercentOutput, 0);
        }
        if(player2.getRawButton(5)) {
            adjustingPiston.set(true);
        } else {
            adjustingPiston.set(false);
        }
    }
}