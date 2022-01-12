// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.networktables.*;

/** An example command that uses an example subsystem. */
public class AlignToTargetCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Drivetrain driveSubsystem;
    private final Navigation navSubsystem;
    private final Vision visionSubsystem;

    private double desiredHeading;
    private double actualHeading;
    //private double visionAngle;

    private double alignStartTime;

    final double targetControllerKp = 0.02;
    final double targetControllerKi = 0.02;
    final double targetControllerKd = 0.004;

    final double targetControllerPeriod = 1.0 / 50.0;
    PIDController targetController;

    private static final double ErrorTolerance = 0.1;
    private static final int MaxErrorsInHistory = 3;
    private static final double AlignmentTimeout = 5000.0;
    private ArrayList<Double> errorHistory;

    private boolean targetNeverSeen;

    /**
     * Creates a new Command.
     *
     * @param subsystem
     *                      The subsystem used by this command.
     */
    public AlignToTargetCommand(Drivetrain driveSubsystem, Navigation navSubsystem, Vision visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.navSubsystem = navSubsystem;
        this.visionSubsystem = visionSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);

        errorHistory = new ArrayList<Double>();
        targetController = new PIDController(targetControllerKp, targetControllerKi, targetControllerKd);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        if (visionSubsystem.targetIsVisible()) {
            targetNeverSeen = false;
            // Get the angle to the target relative to the front of the robot
            desiredHeading = (visionSubsystem.getAngleToTarget() * Constants.RadiansToDegrees) + 4.8;

            // Reset the nav system so that the current heading is zero
            navSubsystem.reset();

            // reset the PID
            targetController.reset();

            // clear the error history
            errorHistory.clear();

            // record our start time so we know how long we've been trying to align
            alignStartTime = System.currentTimeMillis();
        } else {
            targetNeverSeen = true;
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // compute the error which is how far away we are from the desired heading
        actualHeading = navSubsystem.getHeading();
        // if (visionSubsystem.targetIsVisible()) {
        //     visionAngle = visionSubsystem.getAngleToTarget() * Constants.RadiansToDegrees;
        // }
        double error = desiredHeading - actualHeading;

        // save the error so we can use it later to see if we're done
        addErrorToHistory(error);

        // calculate how much input to give to the drive subsystem to reduce the error
        double output = targetController.calculate(error);

        // turn the robot
        driveSubsystem.drive(0.0, -output, false);

        NetworkTableInstance.getDefault().getEntry("auto_align/desired_heading").setDouble(desiredHeading);
        NetworkTableInstance.getDefault().getEntry("auto_align/actual_heading").setDouble(actualHeading);
        NetworkTableInstance.getDefault().getEntry("auto_align/target_visible")
                .setBoolean(visionSubsystem.targetIsVisible());
        NetworkTableInstance.getDefault().getEntry("auto_align/vision_angle_to_target")
                .setDouble(visionSubsystem.getAngleToTarget() * Constants.RadiansToDegrees);
        NetworkTableInstance.getDefault().getEntry("auto_align/error").setDouble(error);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        if (targetNeverSeen) {
            return true;
        } else {
            // We're done if this has been taking too long.
            if (System.currentTimeMillis() - alignStartTime >= AlignmentTimeout) {
                return true;
            }

            // We're done if the errors are all smaller than the acceptable value
            for (Double d : errorHistory) {
                if (Math.abs(d) > ErrorTolerance) {
                    return false;
                }
            }
            return true;
        }
    }

    private void addErrorToHistory(double error) {
        // Add an error to the history. We're only keeping a limited number of history
        // values, so delete
        // the oldest one if we already have enough.
        errorHistory.add(error);
        if (errorHistory.size() > MaxErrorsInHistory) {
            errorHistory.remove(0);
        }

    }

}
