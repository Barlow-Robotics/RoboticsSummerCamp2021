/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
    private final Drivetrain m_drive;
    private final double m_distance;
    private final double m_speed;

    private boolean countersBackToZero;

    /**
     * Creates a new DriveDistance.
     *
     * @param inches
     *                   The number of inches the robot will drive
     * @param speed
     *                   The speed at which the robot will drive
     * @param drive
     *                   The drive subsystem on which this command will run
     */
    public DriveDistance(double inches, double speed, Drivetrain drive) {
        m_distance = inches;
        m_speed = speed;
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        countersBackToZero = false;
        m_drive.ResetDistance();
        m_drive.setBrakeMode();
        // m_drive.drive(m_speed, 0, false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_drive.GetDistanceInInches() == 0) {
            countersBackToZero = true;
        }
        if (countersBackToZero) {
            m_drive.drive(-m_speed, 0, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, false);
        m_drive.setDefaultNeutralMode();
    }

    @Override
    public boolean isFinished() {
        if (countersBackToZero) {
            double distance = m_drive.GetDistanceInInches();
            boolean result = Math.abs(distance) >= m_distance;
            if (result == true) {
                int wpk = 1;
            }
            return result;
        } else {
            return false;
        }
    }

}