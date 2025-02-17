package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.jni.CANSparkJNI;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DebuggingConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SwervePhysicalConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPos_UsingEncoder extends Command {
    // Create subsystem
    private final SwerveSubsystem swerveSubsystem;

    // Initialize movement variables
    private final double xSpd, ySpd, turningSpd;
    private final boolean fieldOriented;
    private double finalPose;
    private final double threshold = 0.05;
    private final int index;
    private Transform2d translation;
    
    // Limiting movement
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    /**
     * Contructs a Command to control the swerve via joystick
     * @param swerveSubsystem subsystem that controls the swerve
     * @param xSpdFunction x speed (left and right)
     * @param ySpdFunction y speed (forward and backwards)
     * @param turningSpdFunction turning speed (rotation) not angle control
     * @param fieldOrientedFunction field orientation (true for field orientated, false for robot orientated)
     */
    public DriveToPos_UsingEncoder(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        // tSets Movement values
        xSpd = -0.4; // inversed bc when
        ySpd = 0.0; 
        turningSpd = 0.0;
        index = 3;
        
        fieldOriented = false;

        translation = new Transform2d(5, 0 , new Rotation2d(0));
        this.xLimiter = new SlewRateLimiter(SwervePhysicalConstants.TELE_DRIVE_MAX_ACCELERATION_UNIT_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(SwervePhysicalConstants.TELE_DRIVE_MAX_ACCELERATION_UNIT_PER_SECOND);
        this.turningLimiter = new SlewRateLimiter(SwervePhysicalConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNIT_PER_SECOND);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        finalPose = (swerveSubsystem.getSwerveModulePosistion()[index].distanceMeters) + translation.getX();
        SmartDashboard.putString("Drive Mode", "Default / Field Oriented"); // Helps understand which command swerve drive is using
    }

    @Override
    public void execute() {
        
        // 1. Get real-time joystick inputs
        double xSpeed = IOConstants.DRIVER_X_AXIS_INVERTED * xSpd;
        double ySpeed = IOConstants.DRIVER_Y_AXIS_INVERTED * ySpd;
        double turningSpeed = IOConstants.DRIVER_ROT_AXIS_INVERTED * turningSpd;

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > IOConstants.DRIVER_DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > IOConstants.DRIVER_DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > IOConstants.DRIVER_DEADBAND ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * SwervePhysicalConstants.TELE_DRIVE_MAX_SPEED_METER_PER_SECOND;
        ySpeed = yLimiter.calculate(ySpeed) * SwervePhysicalConstants.TELE_DRIVE_MAX_SPEED_METER_PER_SECOND;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * SwervePhysicalConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIAN_PER_SECOND;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = SwervePhysicalConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);

        // Debuging
        if(DebuggingConstants.SWERVE_DRIVE_DEBUG) {
            SmartDashboard.putString("Joystick", "X : " + xSpd + "Y : " + ySpd + " Theta : " + turningSpd);
            SmartDashboard.putString("ChassisSpeeds", "X : " + chassisSpeeds.vxMetersPerSecond + "Y : " + chassisSpeeds.vyMetersPerSecond + " Theta : " + chassisSpeeds.omegaRadiansPerSecond);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        double current = (swerveSubsystem.getSwerveModulePosistion()[index].distanceMeters);
        double distance = current - finalPose;
        return Math.abs(distance) < threshold;
    }
}
