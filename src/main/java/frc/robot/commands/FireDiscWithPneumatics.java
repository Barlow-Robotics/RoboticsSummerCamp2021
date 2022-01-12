// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.PneumaticFeeder;
import frc.robot.subsystems.Thrower;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FireDiscWithPneumatics extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final PneumaticFeeder feeder;
    private final Thrower thrower;

    private boolean feederStarted = false;
    private long feederStartTime = 0;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem
     *                      The subsystem used by this command.
     */
    public FireDiscWithPneumatics(PneumaticFeeder f, Thrower t) {
        feeder = f;
        thrower = t;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(feeder);
        addRequirements(thrower);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // start shooter spinning
        thrower.StartSpinning();
        feederStarted = false;
        feederStartTime = System.currentTimeMillis(); // wpk
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!feederStarted) {
            if (thrower.IsUpToSpeed()) {
                // System.out.println("Motor is up to speed, starting feed") ;
                feeder.StartFeeding();
                feederStarted = true;
                feederStartTime = System.currentTimeMillis();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // retract the feeder
        feeder.StopFeeding();
        feederStarted = false;

        // stop the thrower
        thrower.StopSpinning();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        long elapsedTime = System.currentTimeMillis() - feederStartTime;
        if (feederStarted) {
            if (elapsedTime >= Constants.Thrower.Extend_Time) {
                // System.out.println("feed time ELAPSED") ;
                return true;
            } else {
                // System.out.println("feed time waiting") ;
                return false;
            }
        }
        return false;
    }
}
