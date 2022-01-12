package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.geometry.Pose2d;


/** Represents a differential drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

    WPI_TalonFX rightMotor = new WPI_TalonFX(20);
    WPI_TalonFX leftMotor = new WPI_TalonFX(10);

    private DifferentialDrive theDrive = new DifferentialDrive(leftMotor, rightMotor);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    private Navigation navSubSystem;

    /**
     * Constructs a differential drive object. Sets the encoder distance per pulse
     * and resets the gyro.
     */
    public Drivetrain(Navigation nav) {
        rightMotor.setInverted(TalonFXInvertType.CounterClockwise); // !< Update this
        setDefaultNeutralMode();
        navSubSystem = nav;
        m_odometry = new DifferentialDriveOdometry(navSubSystem.getRotation2d());
        ResetDistance();
        CreateNetworkTableEntries();
    }

    private void CreateNetworkTableEntries() {
        NetworkTableInstance.getDefault().getEntry("drive/left_motor_distance").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/right_motor_distance").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rotation").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/leftSpeed").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rightSpeed").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/xSpeed").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rot").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/arcadeDrive").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/tankDrive").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/leftVolts").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rightVolts").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/pose/x").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/pose/y").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/pose/rotation").setDouble(0.0);

    }

    public void setDefaultNeutralMode() {
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setCoastMode() {
        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void setBrakeMode() {
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void ResetDistance() {
        rightMotor.setSelectedSensorPosition(0);
        leftMotor.setSelectedSensorPosition(0);
    }

    public double GetDistanceInInches() {
        double averagePosition 
             = (-leftMotor.getSelectedSensorPosition() + rightMotor.getSelectedSensorPosition()) / 2.0 ;
        double distance = (averagePosition / Constants.Drive.Counts_Per_Revolution)
                * Constants.Drive.Inches_Per_Revolution;
        NetworkTableInstance.getDefault().getEntry("drive/distance_in_inches").setDouble(distance);
        return distance;
    }

    private double getLeftDistance() {
        double d = (leftMotor.getSelectedSensorPosition() / Constants.Drive.Counts_Per_Revolution)
                * Constants.Drive.Meters_Per_Revolution;
        return (d);
    }

    private double getRightDistance() {
        double d = (-rightMotor.getSelectedSensorPosition() / Constants.Drive.Counts_Per_Revolution)
                * Constants.Drive.Meters_Per_Revolution;
        return (d);
    }

    private double getLeftSpeed() {
        double s = leftMotor.getSelectedSensorVelocity() * 10.0 * Constants.Drive.Meters_Per_Count;
        return (s);
    }

    private double getRightSpeed() {
        double s = -rightMotor.getSelectedSensorVelocity() * 10.0 * Constants.Drive.Meters_Per_Count;
        return (s);
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
    }

    public void periodic() {

        NetworkTableInstance.getDefault().getEntry("drive/left_motor_distance").setDouble(getLeftDistance());
        NetworkTableInstance.getDefault().getEntry("drive/right_motor_distance").setDouble(getRightDistance());
        NetworkTableInstance.getDefault().getEntry("drive/rotation").setDouble(navSubSystem.getRotation2d().getDegrees());
        NetworkTableInstance.getDefault().getEntry("drive/leftSpeed").setDouble(getLeftSpeed());
        NetworkTableInstance.getDefault().getEntry("drive/rightSpeed").setDouble(getRightSpeed());

        // Update the odometry in the periodic block
        m_odometry.update( navSubSystem.getRotation2d(), getLeftDistance(), getRightDistance());
        NetworkTableInstance.getDefault().getEntry("drive/pose/x").setDouble(m_odometry.getPoseMeters().getX());
        NetworkTableInstance.getDefault().getEntry("drive/pose/y").setDouble(m_odometry.getPoseMeters().getY());
        NetworkTableInstance.getDefault().getEntry("drive/pose/rotation")
                .setDouble(m_odometry.getPoseMeters().getRotation().getDegrees());

    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }


    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose
     *                 The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, navSubSystem.getRotation2d());
        System.out.println("Odometry was reset");
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double rot, boolean squareInputs) {
        NetworkTableInstance.getDefault().getEntry("drive/xSpeed").setDouble(rightMotor.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/rot").setDouble(rightMotor.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/arcadeDrive").setDouble(100.0);
        NetworkTableInstance.getDefault().getEntry("drive/tankDrive").setDouble(0.0);
        theDrive.arcadeDrive(xSpeed, rot, squareInputs);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts
     *                       the commanded left output
     * @param rightVolts
     *                       the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        NetworkTableInstance.getDefault().getEntry("drive/leftVolts").setDouble(leftVolts);
        NetworkTableInstance.getDefault().getEntry("drive/rightVolts").setDouble(rightVolts);
        NetworkTableInstance.getDefault().getEntry("drive/arcadeDrive").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/tankDrive").setDouble(100.0);

        // wpk this is set the same as the wpi example
        leftMotor.setVoltage(leftVolts);
        rightMotor.setVoltage(-rightVolts);

        theDrive.feed();
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        leftMotor.setSelectedSensorPosition(0.0);
        rightMotor.setSelectedSensorPosition(0.0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput
     *                      the maximum output to which the drive will be
     *                      constrained
     */
    public void setMaxOutput(double maxOutput) {
        theDrive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        navSubSystem.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return navSubSystem.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return navSubSystem.getRate();
    }
}

