// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics; 


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double RadiansToDegrees = 180.0 / Math.PI ;
    public static final double DegreesToRadians = Math.PI / 180.0 ;
    public static final double Inches_Per_Foot = 12.0 ;
    public static final double InchesToMeters = 0.0254 ;


    public static final class Drive{
        public static final int Counts_Per_Revolution = 21300 ;
        public static final double Wheel_Diameter = 6.0 ;
        public static final double Inches_Per_Revolution = Math.PI * Wheel_Diameter ;
        public static final double Meters_Per_Revolution = Inches_Per_Revolution * InchesToMeters ;
        public static final double Meters_Per_Count = Meters_Per_Revolution / Counts_Per_Revolution ; 

        public static final double ksVolts = 0.561 ;
        public static final double kvVoltSecondsPerMeter = 2.45 ;
        public static final double kaVoltSecondsSquaredPerMeter = 0.187 ;
        public static final double kPDriveVel = 2.1 ;

        public static final double kTrackwidthMeters = 0.66;  //wpk need to check this value
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

    }


    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;        
    }

    public static final class Thrower {
        public static final int Counts_Per_Revolution = 2048 ;
        public static final int Max_Counts_Per_Second = 21000 ; // This is about 6000 Rpms
        public static final int Desired_Speed = (int) (1.0 * (double)Max_Counts_Per_Second) ; // Desired speed in counts / second
        public static final long Extend_Time = 500 ; // milliseconds
    }


    public static final class Feeder {
        public static final int Extend_solenoid = 0 ;
        public static final int Retract_Solenoid = 1 ;
    }


    public static final class Vision {
        public static final int Relay_Port = 4 ;
    }


    public final class Logitech_F310_Controller {

        // Constants for Axes
        public static final int Left_Stick_X = 0 ;
        public static final int Left_Stick_Y = 1 ;
        public static final int Left_Trigger = 2 ;
        public static final int Right_Trigger = 3 ;
        public static final int Right_Stick_X = 4 ;
        public static final int Right_Stick_Y = 5 ;

        // Constants for buttons
        public static final int Button_A = 1 ;
        public static final int Button_B = 2 ;
        public static final int Button_X = 3 ;
        public static final int Button_Y = 4 ;
        public static final int Left_Bumper = 5 ;
        public static final int Right_Bumper = 6 ;
        public static final int Back_Button = 7 ;
        public static final int Start_Button = 8 ;
        public static final int Left_Stick = 9 ;
        public static final int Right_Stick = 10 ;
        
    }


    public final class Playstation_Controller {

        // Constants for Axes
        public static final int Left_Stick_X = 0 ;
        public static final int Left_Stick_Y = 1 ;
        public static final int Left_Trigger = 2 ;
        public static final int Right_Trigger = 3 ;
        public static final int Right_Stick_X = 4 ;
        public static final int Right_Stick_Y = 5 ;

        // Constants for buttons
        public static final int Button_A = 1 ;
        public static final int Button_B = 2 ;
        public static final int Button_X = 3 ;
        public static final int Button_Y = 4 ;
        public static final int Left_Bumper = 5 ;
        public static final int Right_Bumper = 6 ;
        public static final int Back_Button = 7 ;
        public static final int Start_Button = 8 ;
        public static final int Left_Stick = 9 ;
        public static final int Right_Stick = 10 ;
        
    }


}
