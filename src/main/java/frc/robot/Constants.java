// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class IntakeConstants {
    public static final int k_INTAKE_MOTOR_ID = 0;
    public static final int k_DJ_MOTOR_ID = 0;

    public static final int k_INTAKE_MOTOR_CURRENT_LIMIT = 0;
    public static final int k_DJ_MOTOR_CURRENT_LIMIT = 0;

    public static final int k_FWD_CHANNEL = 0;
    public static final int k_REV_CHANNEL = 0;

    public static final int BEAMBREAK_CHANNEL = 0;
    public static final int BUTTON_CHANNEL = 0;

    public static final int GAME_PIECE_THRESHOLD = 93; // (blue)
    public static final int DISTANCE_THRESHOLD = 0; //TODO: find
  }

    public static final class SwerveConstants {
      public enum Modules {
        ;
        public static final int FRONT_LEFT = 0;
        public static final int FRONT_RIGHT = 1;
        public static final int BACK_LEFT = 2;
        public static final int BACK_RIGHT = 3;
      }

      public static final int[] kDriveMotorId = {18, 12, 16, 14};
      public static final int[] kSpinningMotorId = {17, 11, 15, 13};
      public static final boolean[] kDriveMotorReversed = {false, false, false, false};
      public static final boolean[] kSpinningMotorReversed = {false, false, false, false};
      public static final int[] kAbsEncoderChannel = {1, 0, 2, 3};
      public static final double[] kOffsetAngle = {
            0.809,
            0.285,
            0.393,
            0.349
      };

      public static final double kTolerance = 0.1;
      public static final double kDeadband = 0.25;

      public static final double kTrackWidth = 0.5842; // m
      public static final double kWheelBase = 0.5842; // m
      public static final SwerveDriveKinematics kSwerveKinematics =
            new SwerveDriveKinematics(
                  new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                  new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                  new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                  new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

      public static final double kPhysicalMaxSpeedMetersPerSecond = 5;//TODO find
      public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;//TODO find
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;//TODO find

      // intentional limitations

      //    public static double kSpeedPercantageLimit = 75; // %
      public static double kMaxDriveSpeed = kPhysicalMaxSpeedMetersPerSecond; // m/s
      public static double kMaxDriveTurningSpeed = kPhysicalMaxAngularSpeedRadiansPerSecond;// rad/s
      public static double kMaxTurningAcceleration = Math.PI; // rad/s^2

      //unclear values
      public static final double kMaxDriveAccelerationUnitsPerSecond = 3;
      public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;

      // autonomous constants
      public static final TrajectoryConfig kConfig =
            new TrajectoryConfig(
                  kMaxDriveSpeed,
                  kMaxAccelerationMetersPerSecondSquared)
                  .setKinematics(kSwerveKinematics);

      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                  kMaxDriveTurningSpeed,
                  kMaxTurningAcceleration);

      public static final double kPXAuto = 0; //TODO: find
      public static final double kPYAuto = 0; //TODO: find
      public static final double kPThetaAuto = 0.75; //TODO: find
    }

    public static final class ModuleConstants {
      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
      public static final double kDriveMotorGearRatio = 1 / 8.14;
      public static final double kDriveEncoderRotationsToMeters = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kDriveEncoderRPMToMeterPerSec = kDriveEncoderRotationsToMeters / 60;
      public static final double kTurningMotorGearRatio = 1 / 21.4285714;
      public static final double kTurningEncoderRotationsToRadians = kTurningMotorGearRatio * 2 * Math.PI;
      public static final double kTurningEncoderRPMToRadiansPerSec = kTurningEncoderRotationsToRadians / 60;
      public static final double kPTurning = 0.5;
    }

    public final class ClawConstants {
      public static final int FORWARD_CHANNEL = 0;
      public static final int REVERSE_CHANNEL = 0;
      public static final int BEAMBREAK_CHANNEL = 0;
      public static final int BUTTON_CHANNEL = 0;
    }
  }
