// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot ;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.AlignToTargetCommand;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.FindTarget;
import frc.robot.commands.AlignToTargetCommand;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.FindTargetAndThrow;
import frc.robot.commands.FireDiscWithPneumatics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.PneumaticFeeder;
import frc.robot.subsystems.Thrower;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.geometry.*;

//import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public static final int kDriverControllerPort = 1; 

    // The driver's controller
    Joystick m_driverController = new Joystick(kDriverControllerPort);

    // The robot's subsystems and commands are defined here...
    public final Navigation navSubsystem = new Navigation();
    public final Drivetrain drive = new Drivetrain(navSubsystem);
    public final Vision visionSubsystem = new Vision();
    public final PneumaticFeeder pneumaticFeeder = new PneumaticFeeder();
    public final Thrower thrower = new Thrower();

    private final AlignToTargetCommand alignCommand = new AlignToTargetCommand(drive, navSubsystem, visionSubsystem);
    private final DriveDistance driveDistanceCommand = new DriveDistance(120, 0.5, drive);
    private final JoystickButton targetAlignButton 
        = new JoystickButton(m_driverController, Constants.Logitech_F310_Controller.Left_Bumper);

    private final FireDiscWithPneumatics shootCommand = new FireDiscWithPneumatics(pneumaticFeeder, thrower);
    private final JoystickButton shootButton = new JoystickButton(m_driverController,
            Constants.Logitech_F310_Controller.Right_Bumper);

    private final JoystickButton headingResetButton = new JoystickButton(m_driverController,
            Constants.Logitech_F310_Controller.Button_B);

    private Command autonomousCommand = null;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        drive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand( // new instance
                        () -> {
                            double x = -m_driverController.getRawAxis(Constants.Logitech_F310_Controller.Left_Stick_Y);
                            double yaw = m_driverController
                                    .getRawAxis(Constants.Logitech_F310_Controller.Right_Stick_X);

                            // fancy exponential formulas to shape the controller inputs to be flat when only 
                            // pressed a little, and ramp up as stick pushed more.
                            double speed = 0.0;
                            if (x != 0) {
                                speed = (Math.abs(x) / x) * (Math.exp(-400.0 * Math.pow(x / 3.0, 4.0)))
                                        + (-Math.abs(x) / x);
                            }
                            double turn = 0.0;
                            if (yaw != 0) {
                                turn = (Math.abs(yaw) / yaw) * (Math.exp(-400.0 * Math.pow(yaw / 3.0, 4.0)))
                                        + (-Math.abs(yaw) / yaw);
                            }
                            // The turn input results in really quick movement of the bot, so
                            // let's reduce the turn input and make it even less if we are going faster
                            // This is a simple y = mx + b equation to adjust the turn input based on the
                            // speed.
                            turn = turn * ( -0.4*Math.abs(speed) + 0.5) ;

                            drive.drive(-speed, -turn * 0.4, false);
                        }, drive));

        autonomousCommand = GetPathFollowingCommand();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        targetAlignButton.whenPressed(alignCommand).whenReleased(new InstantCommand(() -> {
            alignCommand.cancel();
            System.out.println("Canceled align command");

        }, thrower));

        shootButton.whenPressed(shootCommand).whenReleased(new InstantCommand(() -> {
            shootCommand.cancel();
        }, thrower));

        headingResetButton.whenPressed(new InstantCommand(() -> {
            navSubsystem.reset();
        }, navSubsystem));

    }

    private Command GetPathFollowingCommand() {
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.Drive.ksVolts, Constants.Drive.kvVoltSecondsPerMeter,
                        Constants.Drive.kaVoltSecondsSquaredPerMeter),

                Constants.Drive.kDriveKinematics, 10);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.Drive.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        Trajectory exampleTrajectory = null;
        String trajectoryJSON = "./paths/FirstLeg.wpilib.json";

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        if (exampleTrajectory == null) {
            // An example trajectory to follow. All units in meters.
            exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, new Rotation2d(0)),
                    List.of(
                    ), new Pose2d(3, 0, new Rotation2d(0.0)), 
                    config);
        }

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, drive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.Drive.ksVolts, Constants.Drive.kvVoltSecondsPerMeter,
                        Constants.Drive.kaVoltSecondsSquaredPerMeter),
                Constants.Drive.kDriveKinematics, drive::getWheelSpeeds,
                new PIDController(Constants.Drive.kPDriveVel, 0, 0),
                new PIDController(Constants.Drive.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drive::tankDriveVolts, drive);

        // Reset odometry to the starting pose of the trajectory.
        drive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return null ;
        // return driveDistanceCommand ;
        // return new FindTarget(this.drive, this.visionSubsystem, this.navSubsystem) ;
        // return new FireDiscWithPneumatics(this.pneumaticFeeder, this.thrower) ;
        // return new FindTargetAndThrow(drive, visionSubsystem, thrower,
        // pneumaticFeeder, navSubsystem) ;

        return autonomousCommand;

    }
}
