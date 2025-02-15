// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Class used to control debugging info
  public static class DebuggingConstants {
    public static final boolean IS_DEBUG_MASTER = true; // false to turn off all debuging; true to enable but need to
                                                        // turn on individual debuging

    public static final boolean SWERVE_DRIVE_DEBUG = true && IS_DEBUG_MASTER; // Debugging for swerve drive like
                                                                              // printing certain values to smart
                                                                              // dashboard
    public static final boolean IS_IN_AIR = true && IS_DEBUG_MASTER; // Used to help switch between testing in air and
                                                                     // on ground
  }

  public static class SwervePhysicalConstants {
    // Chasis
    public static final double TRACK_WIDTH = Units.inchesToMeters(19); // Distance between right and left wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(19); // Distance between front and back wheels
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Back Right
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2) // Back Left
    );

    // Wheel
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 8.41;
    public static final double TURN_MOTOR_GEAR_RATIO = 1 / 12.8;
    public static final double DRIVE_ENCODER_ROTATION_TO_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double TURN_ENCODER_ROTATION_TO_RADIANS = TURN_MOTOR_GEAR_RATIO * 2 * Math.PI;
    public static final double DRIVE_ENCODER_RPM_TO_METER_PER_SECOND = DRIVE_ENCODER_ROTATION_TO_METER / 60;
    public static final double TURN_ENCODER_RPM_TO_METER_PER_SECOND = TURN_ENCODER_ROTATION_TO_RADIANS / 60;

    // Physical Max
    public static final double PHYSICAL_MAX_SPEED_METER_PER_SECOND = 10; // Modify
    public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIAN_PER_SECOND = 3 * 2 * Math.PI; // Modify

    public static final double TELE_DRIVE_MAX_SPEED_METER_PER_SECOND = PHYSICAL_MAX_SPEED_METER_PER_SECOND * 0.5; // 25%
                                                                                                                  // power
                                                                                                                  // //
                                                                                                                  // Modify
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIAN_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIAN_PER_SECOND
        * 0.25; // 25% power // Modify
    public static final double TELE_DRIVE_MAX_ACCELERATION_UNIT_PER_SECOND = 3; // Modify
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNIT_PER_SECOND = 3; // Modify

    // Motor Names
    public enum MotorLocation {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    };
  }

  // Class for the drive modules of swerve
  public static class SwerveDriveConstants {
    // Motor IDs
    public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 2; // Check 2
    public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 4; // Check 4
    public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 6; // Check 6
    public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 8; // Check 8

    public static final boolean BACK_LEFT_DRIVE_REVERSED = true; // Check / Modify
    public static final boolean FRONT_LEFT_DRIVE_REVERSED = true; // Check / Modify
    public static final boolean FRONT_RIGHT_DRIVE_REVERSED = false; // Check / Modify
    public static final boolean BACK_RIGHT_DRIVE_REVERSED = false; // Check / Modify

    public static final double FREESPEED_RPS = (12.5 / (Units.metersToFeet(SwervePhysicalConstants.WHEEL_DIAMETER) * Math.PI)) 
                                                * SwervePhysicalConstants.DRIVE_MOTOR_GEAR_RATIO;
SwerveDriveConstants.;
    public static final double k_DriveWheelFreeSpeedRps = FREESPEED_RPS / 8.14;

    
    // PID Values
    public static final boolean IS_USING_PID_DRIVE = false;

    public static final double P_DRIVE = 0.0000000001; // originally 0.000000005
    public static final double I_DRIVE = 0; // originally 0
    public static final double D_DRIVE = 0; // originally 0.4
    public static final double FF_DRIVE = 0; // originally 0
    public static final double IZONE_DRIVE = 0; // originally 0
    public static final double MIN_DRIVE = -1; // originally -1
    public static final double MAX_DRIVE = 1; // originally -1

    public static final double FEEDFORWARD_S_DRIVE = 0; // Change
    public static final double FEEDFORWARD_V_DRIVE = 0; // Change
    public static final double FEEDFORWARD_A_DRIVE = 0; // Change, but can leave at zero if needed
  }

  // Class for the turn modules of swerve
  public static class SwerveTurnConstants {
    // Motor IDs
    public static final int BACK_LEFT_TURN_MOTOR_PORT = 1; // Right // 0.22 // 0.28, 0.4
    public static final int FRONT_LEFT_TURN_MOTOR_PORT = 3; // Right // 0.20 // 0.267, 0.45
    public static final int FRONT_RIGHT_TURN_MOTOR_PORT = 5; // Right // 0.22 // 0.36, 0.45
    public static final int BACK_RIGHT_TURN_MOTOR_PORT = 7; // Right // 0.225 // 0.366, 0.38

    public static final boolean BACK_LEFT_TURN_REVERSED = true; // Check / Modify
    public static final boolean FRONT_LEFT_TURN_REVERSED = true; // Check / Modify
    public static final boolean FRONT_RIGHT_TURN_REVERSED = true; // Check / Modify
    public static final boolean BACK_RIGHT_TURN_REVERSED = true; // Check / Modify

    // Encoder Values
    public static final int BACK_LEFT_TURN_ABSOLUTE_ENCODER_PORT = 12; // Check
    public static final int FRONT_LEFT_TURN_ABSOLUTE_ENCODER_PORT = 9; // Check
    public static final int FRONT_RIGHT_TURN_ABSOLUTE_ENCODER_PORT = 10; // Check
    public static final int BACK_RIGHT_TURN_ABSOLUTE_ENCODER_PORT = 11; // Check

    public static final boolean BACK_LEFT_TURN_ABSOLUTE_ENCODER_REVERSED = true; // Check / Modify
    public static final boolean FRONT_LEFT_TURN_ABSOLUTE_ENCODER_REVERSED = true; // Check / Modify
    public static final boolean FRONT_RIGHT_TURN_ABSOLUTE_ENCODER_REVERSED = true; // Check / Modify
    public static final boolean BACK_RIGHT_TURN_ABSOLUTE_ENCODER_REVERSED = true; // Check / Modify

    public static final double BACK_LEFT_TURN_ABSOLUTE_ENCODER_OFFSET_ROTATION = 0.000244; // 0.311279; //0.314453; //
                                                                                           // 0.314209; // Check /
                                                                                           // Modify //0.312500,
                                                                                           // 0.000244
    public static final double FRONT_LEFT_TURN_ABSOLUTE_ENCODER_OFFSET_ROTATION = 0.000244; // 0.101074;
                                                                                            // //0.095215;//0.584961;
                                                                                            // // Check / Modify
                                                                                            // //0.097656, 0.000000
    public static final double FRONT_RIGHT_TURN_ABSOLUTE_ENCODER_OFFSET_ROTATION = 0.000244; // 0.831055; //
                                                                                             // 0.824951;//0.329834; //
                                                                                             // Check / Modify
                                                                                             // //0.828369, 0.000000
    public static final double BACK_RIGHT_TURN_ABSOLUTE_ENCODER_OFFSET_ROTATION = 0.000244; // 0.090088; //
                                                                                            // 0.076660;//0.604980; //
                                                                                            // Check / Modify
                                                                                            // //0.077637, -0.000244

    public static final double BACK_LEFT_TURN_ABSOLUTE_ENCODER_OFFSET_RADIANS = BACK_LEFT_TURN_ABSOLUTE_ENCODER_OFFSET_ROTATION
        * 2 * Math.PI; // Check / Modify
    public static final double FRONT_LEFT_TURN_ABSOLUTE_ENCODER_OFFSET_RADIANS = FRONT_LEFT_TURN_ABSOLUTE_ENCODER_OFFSET_ROTATION
        * 2 * Math.PI; // Check / Modify
    public static final double FRONT_RIGHT_TURN_ABSOLUTE_ENCODER_OFFSET_RADIANS = FRONT_RIGHT_TURN_ABSOLUTE_ENCODER_OFFSET_ROTATION
        * 2 * Math.PI; // Check / Modify
    public static final double BACK_RIGHT_TURN_ABSOLUTE_ENCODER_OFFSET_RADIANS = BACK_RIGHT_TURN_ABSOLUTE_ENCODER_OFFSET_ROTATION
        * 2 * Math.PI; // Check / Modify

    // PID Values
    public static final double P_BACK_LEFT_TURN = DebuggingConstants.IS_IN_AIR ? 0.5 : 0.28;
    public static final double P_FRONT_LEFT_TURN = DebuggingConstants.IS_IN_AIR ? 0.50 : 0.267;
    public static final double P_FRONT_RIGHT_TURN = DebuggingConstants.IS_IN_AIR ? 0.5 : 0.36;
    public static final double P_BACK_RIGHT_TURN = DebuggingConstants.IS_IN_AIR ? 0.5 : 0.366;

    public static final double I_BACK_LEFT_TURN = DebuggingConstants.IS_IN_AIR ? 0 : 0;
    public static final double I_FRONT_LEFT_TURN = DebuggingConstants.IS_IN_AIR ? 0 : 0;
    public static final double I_FRONT_RIGHT_TURN = DebuggingConstants.IS_IN_AIR ? 0 : 0;
    public static final double I_BACK_RIGHT_TURN = DebuggingConstants.IS_IN_AIR ? 0 : 0;

    public static final double D_BACK_LEFT_TURN = DebuggingConstants.IS_IN_AIR ? 0 : 0;
    public static final double D_FRONT_LEFT_TURN = DebuggingConstants.IS_IN_AIR ? 0 : 0;
    public static final double D_FRONT_RIGHT_TURN = DebuggingConstants.IS_IN_AIR ? 0 : 0;
    public static final double D_BACK_RIGHT_TURN = DebuggingConstants.IS_IN_AIR ? 0 : 0;

    public static final double STATIC_BACK_LEFT_TURN = DebuggingConstants.IS_IN_AIR ? 0 : 0.4;
    public static final double STATIC_FRONT_LEFT_TURN = DebuggingConstants.IS_IN_AIR ? 0 : 0.45;
    public static final double STATIC_FRONT_RIGHT_TURN = DebuggingConstants.IS_IN_AIR ? 0 : 0.45;
    public static final double STATIC_BACK_RIGHT_TURN = DebuggingConstants.IS_IN_AIR ? 0 : 0.38;

  }

  public static final class IOConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0; // Check

    public static final int DRIVER_Y_AXIS = 1; // Check
    public static final int DRIVER_X_AXIS = 0; // Check
    public static final int DRIVER_ROT_AXIS = 4; // Check
    public static final int DRIVER_FIELD_ORIENTED_BUTTON_IDX = 1; // Check

    public static final int DRIVER_Y_AXIS_INVERTED = 1;
    public static final int DRIVER_X_AXIS_INVERTED = -1;
    public static final int DRIVER_ROT_AXIS_INVERTED = 1;

    public static final double DRIVER_DEADBAND = 0.25; // Modify
  }

  public static final class AutoConstants {
    public static final double MAX_SPEED_METER_PER_SECOND = SwervePhysicalConstants.PHYSICAL_MAX_SPEED_METER_PER_SECOND
        * 0.25; // 25% power
    public static final double MAX_ANGULAR_SPEED_RADIAN_PER_SECOND = SwervePhysicalConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIAN_PER_SECOND
        * 0.25; // 10% power
    public static final double MAX_ACCELERATION_METER_PER_SECOND_SQUARED = 3;
    public static final double MAX_ANGULAR_ACCELERATION_RADIAN_PER_SECOND_SQUARED = 3;// Math.PI / 4;
    public static final double P_X_CONTROLLER = 15;// 0.33;//1.5; // Modify
    public static final double P_Y_CONTROLLER = 1.5;// 0.33;//1.5; // Modify
    public static final double P_THETA_CONTROLLER = 3;// 0.33;//3;

    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = //
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIAN_PER_SECOND,
            MAX_ANGULAR_ACCELERATION_RADIAN_PER_SECOND_SQUARED);
  }
}
