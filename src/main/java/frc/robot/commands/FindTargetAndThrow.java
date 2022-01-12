/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.PneumaticFeeder;
import frc.robot.subsystems.Thrower;
import frc.robot.subsystems.Navigation;

public class FindTargetAndThrow extends SequentialCommandGroup {
    private final Drivetrain m_drive;
    private final Vision m_vision;
    private final Thrower m_thrower;
    private final PneumaticFeeder m_feeder;
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
    public FindTargetAndThrow(Drivetrain drive, Vision vision, Thrower thrower, PneumaticFeeder feeder,
            Navigation nav) {
        m_drive = drive;
        m_vision = vision;
        m_thrower = thrower;
        m_feeder = feeder;
        m_navigation = nav;

        this.addCommands(new DriveDistance(120, 0.6, m_drive), new FindTarget(m_drive, m_vision, m_navigation),
                new AlignToTargetCommand(m_drive, m_navigation, m_vision),
                new FireDiscWithPneumatics(m_feeder, m_thrower));

    }

    // @Override
    // public void initialize() {
    // }

    // // Called every time the scheduler runs while the command is scheduled.
    // @Override
    // public void execute() {
    // if ( !m_vision.targetIsVisible()) {
    // m_drive.drive(0.2, 0, false);
    // }
    // }

    // @Override
    // public void end(boolean interrupted) {
    // m_drive.drive(0, 0, false);
    // }

    // @Override
    // public boolean isFinished() {
    // return m_vision.targetIsVisible();
    // }

}