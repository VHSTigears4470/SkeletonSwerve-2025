package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import com.revrobotics.spark.config.SparkMaxConfig;

public final class Configs{
    // Driving Configs
    public static final SparkMaxConfig frontRightDrivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig frontLeftDrivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig backRightDrivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig backLeftDrivingConfig = new SparkMaxConfig();

    // Turning Configs
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig(); 
    
    static {
        double drivingFactor = Constants.SwervePhysicalConstants.WHEEL_DIAMETER * Math.PI
                               * Constants.SwervePhysicalConstants.DRIVE_MOTOR_GEAR_RATIO; // TODO
        double turningFactor = 2 * Math.PI; // TODO
        // double drivingVelocityFeedForward = 1 / (Constants.SwerveDriveConstants.k_DrivingMotorFreeSpeedRps); // TODO

        frontRightDrivingConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .inverted(false);
        frontRightDrivingConfig.encoder 
            .positionConversionFactor(drivingFactor)
            .velocityConversionFactor(drivingFactor / 60.0);
            
        frontLeftDrivingConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .inverted(true);
        frontLeftDrivingConfig.encoder 
            .positionConversionFactor(drivingFactor)
            .velocityConversionFactor(drivingFactor / 60.0);
            
        backRightDrivingConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .inverted(false);
        backRightDrivingConfig.encoder 
            .positionConversionFactor(drivingFactor)
            .velocityConversionFactor(drivingFactor / 60.0);

        backLeftDrivingConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .inverted(true);
        backLeftDrivingConfig.encoder 
            .positionConversionFactor(drivingFactor)
            .velocityConversionFactor(drivingFactor / 60.0);

        turningConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);
        turningConfig.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(turningFactor)
            .velocityConversionFactor(turningFactor / 60.0);
    }
}

