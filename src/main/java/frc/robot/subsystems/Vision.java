package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalOutput;

import edu.wpi.first.networktables.* ;

public class Vision extends SubsystemBase {

    NetworkTableEntry targetAngleEntry ;
    NetworkTableEntry targetVisibleEntry ;

    DigitalOutput ledRelay ; 


    public Vision() {
        ledRelay = new DigitalOutput( Constants.Vision.Relay_Port) ;
        ledRelay.set(false) ;

        targetAngleEntry = NetworkTableInstance.getDefault().getTable("vision").getEntry("targetAngle") ;
        targetVisibleEntry = NetworkTableInstance.getDefault().getTable("vision").getEntry("canSeeTarget") ;
    }



    public boolean targetIsVisible() {
        return targetVisibleEntry.getBoolean(false) ;
    }


    public double getAngleToTarget() {
        return targetAngleEntry.getDouble(0.0);
    }


    public void TurnOnLED() {
        ledRelay.set(true) ;
    }

    
    public void TurnOffLED() {
        ledRelay.set(false) ;
    }



}
