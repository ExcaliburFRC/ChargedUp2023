// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import static java.lang.Math.PI;

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
        public static final int INTAKE_MOTOR_ID = 41;

        public static final int INTAKE_FWD_CHANNEL = 2;
        public static final int INTAKE_REV_CHANNEL = 3;

        public static final int EJECT_FWD_CHANNEL = 0;
        public static final int EJECT_REV_CHANNEL = 1;

        public static double kS = 0.070785;
        public static double kV = 0.12968 / 60;

//        public static double kP = 7.1211E-07;
        public static double kP = 0;

        public static final double LOW_RPM = -(0.1 * 12) / kV;
        public static final double MID_RPM = -(0.3 * 12) / kV;
//        public static final double HIGH_RPM = -(0.65 * 12) / kV;
        public static final double HIGH_RPM = -3500;

        public static final double TOLERANCE = 100;
    }

    public static final class SpindexerConstants{
        public static final int BEAMBREAK_CHANNEL = 0;
        public static final int BUTTON_CHANNEL = 0;

        public static final int SPINDEXER_MOTOR_ID = 0;
        public static final int SPINDEXER_CURRENT_LIMIT = 0; // TODO: calculate
    }

    public static final class ClawConstants {
        public static final int FORWARD_CHANNEL = 0;
        public static final int REVERSE_CHANNEL = 1;

        public enum GamePiece {
            EMPTY,
            CUBE,
            CONE;
        }
    }

    public static final class RollerGripperConstants{
        public static final int INTAKE_BEAMBREAK = 9;
        public static final int RIGHT_ROLLER_MOTOR_ID = 31;
        public static final int LEFT_ROLLER_MOTOR_ID = 32;

    }

    public static final class SwerveConstants {
        public enum Modules {
            // drive ID, spin ID, abs encoder channel, offset angle, drive reversed, angle reversed
            FL(18, 17, 1, 0.093 + 0.25, false, false),
            FR(12, 11, 0, 0.372 + 0.25, false, false),
            BL(16, 15, 2, 0.05, false, false),
            BR(14, 13, 3, 0.570 + 0.25, false, false);

            public int DRIVE_MOTOR_ID;
            public int SPIN_MOTOR_ID;
            public int ABS_ENCODER_CHANNEL;
            public double OFFSET_ANGLE;
            public boolean DRIVE_MOTOR_REVERSED;
            public boolean SPIN_MOTOR_REVERSED;

            public static final int FRONT_LEFT = 0;
            public static final int FRONT_RIGHT = 1;
            public static final int BACK_LEFT = 2;
            public static final int BACK_RIGHT = 3;

            Modules(int DRIVE_MOTOR_ID,
                    int SPIN_MOTOR_ID,
                    int ABS_ENCODER_CHANNEL,
                    double OFFSET_ANGLE,
                    boolean DRIVE_MOTOR_REVERSED,
                    boolean SPIN_MOTOR_REVERSED){
                this.DRIVE_MOTOR_ID = DRIVE_MOTOR_ID;
                this.SPIN_MOTOR_ID = SPIN_MOTOR_ID;
                this.ABS_ENCODER_CHANNEL = ABS_ENCODER_CHANNEL;
                this.OFFSET_ANGLE = OFFSET_ANGLE;
                this.DRIVE_MOTOR_REVERSED = DRIVE_MOTOR_REVERSED;
                this.SPIN_MOTOR_REVERSED = SPIN_MOTOR_REVERSED;
            }
        }

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

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5; //TODO find
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * PI; //TODO find
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; //TODO find

        public static final double RAMP_BALANCE_KP = 0.05;

        // intentional limitations

        public static final double kSpeedPercantageLimit = 95; // %
        public static final double kMaxDriveSpeed = kPhysicalMaxSpeedMetersPerSecond / 100 * kSpeedPercantageLimit; // m/s
        public static final double kMaxDriveTurningSpeed = kPhysicalMaxAngularSpeedRadiansPerSecond / 100 * kSpeedPercantageLimit;// rad/s
        public static final double kMaxTurningAcceleration = PI / 100 * kSpeedPercantageLimit; // rad/s^2

        //unclear values
        public static final double kMaxDriveAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;

        // autonomous constants
        public static final double kp_Theta_TELEOP = 0.0145; //TODO: find
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.14;
        public static final double kDriveEncoderRotationsToMeters = kDriveMotorGearRatio * PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPMToMeterPerSec = kDriveEncoderRotationsToMeters / 60;
        public static final double kTurningMotorGearRatio = 1 / 21.4285714;
        public static final double kTurningEncoderRotationsToRadians = kTurningMotorGearRatio * 2 * PI;
        public static final double kTurningEncoderRPMToRadiansPerSec = kTurningEncoderRotationsToRadians / 60;
        public static final double kPTurning = 0.75;
    }

    public static final class ArmConstants {
        public enum ConeDutyCycle {
            // cone, cube
            LOW(-0.02953, 0), // 0.552
            MID(-0.0326, 0),
            HIGH(-0.05, 0), //1.58
            SHELF(-0.0425, 0), //0.4
            SPINDEXER(0, 0.5),
            INTAKE(0, 1);

            public double dc;
            public double telescope;

            ConeDutyCycle(double dc, double telescope) {
                this.dc = dc;
                this.telescope = telescope;
            }
        }

        private static boolean isAchievableTranslation(Translation2d target) {
            return target.getNorm() >= MINIMAL_LENGTH_METERS && target.getNorm() <= MINIMAL_LENGTH_METERS * 2 &&
                    (target.getAngle().getDegrees() <= PHYSICAL_BACK_MAX_ARM_ANGLE_DEG ||
                            target.getAngle().getDegrees() >= PHYSICAL_FRONT_MAX_ARM_ANGLE_DEG);
        }

        public static final int ANGLE_MOTOR_ID = 21;
        public static final int ANGLE_FOLLOWER_MOTOR_ID = 22;
        public static final int LENGTH_MOTOR_ID = 23;

        public static final int CLOSED_LIMIT_SWITCH_ID = 6;

        public static final int ABS_ANGLE_ENCODER_CHANNEL = 8;

        public static final double RADIUS = 1.5 / 100;
        public static final double ROT_TO_METER = 2 * PI * RADIUS;
        public static final double RPM_TO_METER_PER_SEC = ROT_TO_METER / 60; //link: https://brainly.in/question/3238411
        public static final double MINIMAL_LENGTH_METERS = 0.06175;// m
        public static final double MAXIMAL_LENGTH_METERS = 0.105;// m


        public static final double ABS_ENCODER_OFFSET_ANGLE_DEG = 0.486;
        public static final int PHYSICAL_FRONT_MAX_ARM_ANGLE_DEG = 220;
        public static final int PHYSICAL_BACK_MAX_ARM_ANGLE_DEG = 150;
    }

    public static class Coordinates {
        public enum RampLocations {
            LEFT(new Translation2d(4, 3.481), new Translation2d(12.75, 3.481)),
            MIDDLE(new Translation2d(4, 2.707), new Translation2d(12.75, 2.707)),
            RIGHT(new Translation2d(4, 1.9), new Translation2d(12.75, 1.9));

            public Translation2d blue;
            public Translation2d red;

            RampLocations(Translation2d blue, Translation2d red){
                this.blue = blue;
                this.red = red;
            }
        }

        public static double middleAxisXValue = 8.3;
    }

    public static class LedsConstants {
        public static final int LEDS_PORT = 0;
    }

    // DIO:
    // swerve: 0 - 3
    // claw bb: unknown
    // intake bb: unknown
    // arm:
    //      closed: 6
    //      encoder: 8
}
