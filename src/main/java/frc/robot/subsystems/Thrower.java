package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.*;

public class Thrower extends SubsystemBase {

    WPI_TalonFX motor;

    public Thrower() {
        motor = new WPI_TalonFX(30);
        motor.configFactoryDefault();

        motor.config_kF(0, 0.04871429);
        motor.config_kP(0, 0.0099);
        motor.config_kI(0, 0.0000);
        motor.config_kD(0, 0.0000);

        motor.configClosedloopRamp(0.1);
        motor.setNeutralMode(NeutralMode.Coast);
    }

    public void StartSpinning() {
        motor.set(ControlMode.Velocity, Constants.Thrower.Desired_Speed); // CHANGE
    }

    public void StopSpinning() {
        motor.set(0.0);
    }

    public boolean IsUpToSpeed() {
        return motor.getSelectedSensorVelocity() >= (Constants.Thrower.Desired_Speed * 0.97);
    }

    public double getSpeed() {
        NetworkTableInstance.getDefault().getEntry("thrower/velocity").setDouble(motor.getSelectedSensorVelocity());
        return motor.getSelectedSensorVelocity();
    }

}
