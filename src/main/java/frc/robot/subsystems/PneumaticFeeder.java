package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticFeeder extends SubsystemBase {

    Solenoid extendSolenoid, retractSolenoid;

    Compressor compressor = new Compressor() ;
    
    public PneumaticFeeder() {
        extendSolenoid = new Solenoid(Constants.Feeder.Extend_solenoid);
        retractSolenoid = new Solenoid(Constants.Feeder.Retract_Solenoid);
        StopFeeding();
    }


    public void StartFeeding() {
        extendSolenoid.set(true);
        retractSolenoid.set(false);
    }


    public void StopFeeding() {
        extendSolenoid.set(false);
        retractSolenoid.set(true);
    }

    
}
