package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class Navigation extends SubsystemBase {

    AHRS navX;

    public Navigation() {
        navX = new AHRS(SerialPort.Port.kUSB);
    }

    public double getHeading() {
        // headingEntry.setDouble( navX.getAngle()) ;
        return navX.getAngle();
    }

    public void reset() {
        System.out.println("nav x was reset");
        navX.reset();
    }

    public Rotation2d getRotation2d() {
        Rotation2d temp = new Rotation2d(-navX.getRotation2d().getRadians());
        return temp;
    }

    public double getRate() {
        return navX.getRate();
    }

}
