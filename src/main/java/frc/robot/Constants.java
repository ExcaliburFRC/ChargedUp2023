// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.InterpolatingTreeMap;

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
    public static final int k_SPINDEXER_MOTOR_ID = 0;

    public static final int k_INTAKE_MOTOR_CURRENT_LIMIT = 0;
    public static final int k_DJ_MOTOR_CURRENT_LIMIT = 0;

    public static final int k_FWD_CHANNEL = 0;
    public static final int k_REV_CHANNEL = 0;

    public static final int BEAMBREAK_CHANNEL = 0;
    public static final int BUTTON_CHANNEL = 0;

    public static final int GAME_PIECE_THRESHOLD = 93; // (blue)
    public static final int DISTANCE_THRESHOLD = 0; //TODO: find


    public static final double K_INTAKE_MOTOR_VELOCITY = 0;
  }

  public static final class SwerveConstants {
    public enum Modules {
      ;
      public static final int FRONT_LEFT = 0;
      public static final int FRONT_RIGHT = 1;
      public static final int BACK_LEFT = 2;
      public static final int BACK_RIGHT = 3;
    }

    public static final int[] kDriveMotorId = {17, 11, 15, 13};
    public static final int[] kSpinningMotorId = {18, 12, 16, 14};
    public static final boolean[] kDriveMotorReversed = {false, false, false, false};
    public static final boolean[] kSpinningMotorReversed = {false, false, false, false};
    public static final int[] kAbsEncoderChannel = {1, 0, 2, 3};
    public static final double[] kOffsetAngle = {0.826, 0.038, 0.622, 0.860};

    public static final double kTolerance = 0.05;
    public static final double kDeadband = 0.05;

    public static final double kTrackWidth = 0.56665; // m
    public static final double kWheelBase = 0.56665; // m
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

    public static final double kSpeedPercantageLimit = 50; // %
    public static final double kMaxDriveSpeed = kPhysicalMaxSpeedMetersPerSecond / 100 * kSpeedPercantageLimit; // m/s
    public static final double kMaxDriveTurningSpeed = kPhysicalMaxAngularSpeedRadiansPerSecond / 100 * kSpeedPercantageLimit;// rad/s
    public static final double kMaxTurningAcceleration = Math.PI / 100 * kSpeedPercantageLimit; // rad/s^2

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
    public static final double kPThetaTeleop = 0.0145; //TODO: find
    public static final double kDThetaTeleop = 0.001; //TODO: find
    public static final double kPThetaAuto = 0; //TODO: find
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 8.14;
    public static final double kDriveEncoderRotationsToMeters = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderRPMToMeterPerSec = kDriveEncoderRotationsToMeters / 60;
    public static final double kTurningMotorGearRatio = 1 / 21.4285714;
    public static final double kTurningEncoderRotationsToRadians = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kTurningEncoderRPMToRadiansPerSec = kTurningEncoderRotationsToRadians / 60;
    public static final double kPTurning = 0.6;
  }

    public static final class ClawConstants {
      public static final int FORWARD_CHANNEL = 0;
      public static final int REVERSE_CHANNEL = 0;
      public static final int BEAMBREAK_CHANNEL = 0;
      public static final int BUTTON_CHANNEL = 0;
      public static final int COLOR_SENSOR_CHANNEL = 0;

      public static enum GamePiece{
        EMPTY,
        CUBE,
        CONE;
      }
    }

  public static final class ArmConstants {
    public enum Setpoints {
      SPINDEXER_SET_POINT(0, 0),
      GROUND_INTAKE_SET_POINT(0, 0),
      CONE_LOW_LEVEL_POINT(0, 0),
      CONE_MID_LEVEL_POINT(0, 0),
      CONE_HIGH_LEVEL_POINT(0, 0),
      CUBE_LOW_LEVEL_POINT(0, 0),
      CUBE_MID_LEVEL_POINT(0, 0),
      CUBE_HIGH_LEVEL_POINT(0, 0);

      public final Translation2d translation;

      private boolean isAchievableTranslation(Translation2d target) {
        return target.getNorm() >= MINIMAL_LENGTH_METERS && target.getNorm() <= MINIMAL_LENGTH_METERS * 2 &&
                (target.getAngle().getDegrees() <= PHYSICAL_BACK_MAX_ARM_ANGLE_DEG ||
                        target.getAngle().getDegrees() >= PHYSICAL_FRONT_MAX_ARM_ANGLE_DEG);
      } // TODO: find the max length multiplier

      Setpoints(double x, double y) {
        this.translation = new Translation2d(x,y);
        if(!isAchievableTranslation(translation)) {
          throw new AssertionError("Unattainable setpoint in enum " + this.name());
        };
      }
    }

    public static final int ANGLE_MOTOR_ID = 0;
    public static final int ANGLE_FOLLOWER_MOTOR_ID = 0;
    public static final int LENGTH_MOTOR_ID = 0;

    public static final int UPPER_LIMIT_SWITCH_ID = 0;
    public static final int LOWER_LIMIT_SWITCH_ID = 0;

    public static final int ABS_ANGLE_ENCODER_CHANNEL = 0;

    public static final double RPM_TO_DEG_PER_SEC = 0; // rot/sec
    public static final double ROT_TO_METER = 0; // rot/m
    public static final double RPM_TO_METER_PER_SEC = 0; // rpm/ms
    public static final double MINIMAL_LENGTH_METERS = 0.6; // m

    // Angle control
    public static final double kS_ANGLE = 0;
    public static final double kV_ANGLE = 0;
    public static final InterpolatingTreeMap<Double, Double> kG_ANGLE = new InterpolatingTreeMap<>();
    static {
      // TODO
      // Fill with data points
      // (length, kG)
//        kG.put(0,0);
    }
    public static final double kP_ANGLE = 0;
    public static final double kMaxAngularVelocity = 0;
    public static final double kMaxAngularAcceleration = 0;

    // Length control
    public static final double kS_LENGTH = 0;
    public static final double kV_LENGTH = 0;
    public static final double kG_LENGTH = 0;
    public static final double kP_LENGTH = 0;
    public static final double kMaxLinearVelocity = 0;
    public static final double kMaxLinearAcceleration = 0;

    public static final double ABS_ENCODER_OFFSET_ANGLE_DEG = 0;
    public static final int PHYSICAL_FRONT_MAX_ARM_ANGLE_DEG = 0;
    public static final int PHYSICAL_BACK_MAX_ARM_ANGLE_DEG = 0;

    public static final int ARM_RAMP_RATE = 0;
  }

  public static class CoordinatesConstants {
    public static Translation2d toOppositeAlliance(Translation2d lastPoint) {
      double dis = Math.abs(middleAxisXValue - lastPoint.getX());
      return new Translation2d(
              lastPoint.getX() < middleAxisXValue ? middleAxisXValue + dis : middleAxisXValue - dis,
              lastPoint.getY());
    }

    public static double middleAxisXValue = 0;
  }
  }
