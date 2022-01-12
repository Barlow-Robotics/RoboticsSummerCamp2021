/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Navigation;

public class FindTarget extends CommandBase {
    private final Drivetrain m_drive;
    private final Vision m_vision;
    private final Navigation m_navigation;

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
    public FindTarget(Drivetrain drive, Vision vision, Navigation nav) {
        m_drive = drive;
        m_vision = vision;
        m_navigation = nav;
        addRequirements(drive, nav);
    }

    @Override
    public void initialize() {
        // Reset the nav system so that the current heading is zero
        m_navigation.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!m_vision.targetIsVisible()) {
            m_drive.drive(0.0, 0.2, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, false);
    }

    @Override
    public boolean isFinished() {
        if (m_vision.targetIsVisible()) {
            int wpk = 1;
        }
        if (m_navigation.getHeading() > 360.0) {
            int wpk = 1;
        }
        return m_vision.targetIsVisible() || m_navigation.getHeading() > 360.0;
    }

}