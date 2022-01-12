// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Vision;
import edu.wpi.first.networktables.*;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  NetworkTableEntry headingEntry ;

  NetworkTableEntry frameTime;
  NetworkTableEntry loopTime;


  private long startTime, endTime, duration, previousStartTime;

  


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    headingEntry = NetworkTableInstance.getDefault().getEntry("robot_heading") ;
    headingEntry.forceSetDouble(0.0);

    frameTime = NetworkTableInstance.getDefault().getTable("performance").getEntry("frameTime");
    loopTime = NetworkTableInstance.getDefault().getTable("performance").getEntry("loopTime");

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // // Code to measure performance of the robot code
    // startTime = System.currentTimeMillis();
    // loopTime.forceSetNumber(System.currentTimeMillis() - previousStartTime);
    // previousStartTime = System.currentTimeMillis();
    // //============================================================

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    headingEntry.setDouble( m_robotContainer.navSubsystem.getHeading()) ;

    double speed = m_robotContainer.thrower.getSpeed() ;


    //============================================================
    endTime = System.currentTimeMillis();
    duration = endTime - startTime;
    frameTime.forceSetNumber(duration);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
      m_robotContainer.visionSubsystem.TurnOffLED() ;
  }

  @Override
  public void disabledPeriodic() {
    // Code to measure performance of the robot code
    startTime = System.currentTimeMillis();
    loopTime.forceSetNumber(System.currentTimeMillis() - previousStartTime);
    previousStartTime = System.currentTimeMillis();
    //============================================================

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.visionSubsystem.TurnOnLED() ;

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Code to measure performance of the robot code
    startTime = System.currentTimeMillis();
    loopTime.forceSetNumber(System.currentTimeMillis() - previousStartTime);
    previousStartTime = System.currentTimeMillis();
    //============================================================

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.visionSubsystem.TurnOnLED() ;

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Code to measure performance of the robot code
    startTime = System.currentTimeMillis();
    loopTime.forceSetNumber(System.currentTimeMillis() - previousStartTime);
    previousStartTime = System.currentTimeMillis();
    //============================================================

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.visionSubsystem.TurnOnLED() ;

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Code to measure performance of the robot code
    startTime = System.currentTimeMillis();
    loopTime.forceSetNumber(System.currentTimeMillis() - previousStartTime);
    previousStartTime = System.currentTimeMillis();
    //============================================================

  }
}
