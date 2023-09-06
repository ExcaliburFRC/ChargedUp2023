// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.Color;
import frc.robot.utility.Colors;

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
    public static final class RollerGripperConstants {
        public static final int INTAKE_BEAMBREAK = 7;
        public static final int RIGHT_ROLLER_MOTOR_ID = 31;
        public static final int LEFT_ROLLER_MOTOR_ID = 32;

    }

    public static final class SwerveConstants {
        public enum Modules {
            // drive ID, spin ID, abs encoder channel, offset angle, drive reversed, angle reversed
            FL(18, 17, 1, 0.093 + 0.25, false, false),
            FR(12, 11, 0, 0.372 + 0.25, false, false),
            BL(16, 15, 2, 0.05, false, false),
            BR(14, 13, 3, 0.834, false, false);

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
                    boolean SPIN_MOTOR_REVERSED) {
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

        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(12); //TODO find
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * PI; //TODO find
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; //TODO find

        public static final double RAMP_BALANCE_KP = 0.006;
        public static final double RAMP_BALANCE_KD = 0.002;

        // intentional limitations
        public static final double kSpeedPercantageLimit = 25; // %
        public static final double kMaxDriveSpeed = kPhysicalMaxSpeedMetersPerSecond / 100 * kSpeedPercantageLimit; // m/s
        public static final double kMaxDriveTurningSpeed = kPhysicalMaxAngularSpeedRadiansPerSecond / 100 * kSpeedPercantageLimit;// rad/s
        public static final double kMaxTurningAcceleration = PI / 100 * kSpeedPercantageLimit; // rad/s^2

        //unclear values
        public static final double kMaxDriveAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;

        // autonomous constants
        public static final double kp_Theta = 0.00585;
        public static final double kd_Theta = 0;
        public static final double kp_X = 0.2;
        public static final double kp_Y = 0.2;
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
        public enum Setpoints {
            // cone, cube
            LOW(new Translation2d(0.7, Rotation2d.fromDegrees(130))),//
            MID(new Translation2d(0.72, Rotation2d.fromDegrees(172))),
            HIGH_CHECKPOINT(new Translation2d(MAXIMAL_LENGTH_METERS, Rotation2d.fromDegrees(130))),
            HIGH(new Translation2d(MAXIMAL_LENGTH_METERS, Rotation2d.fromDegrees(192))),
            SHELF_EXTENDED(new Translation2d(0.75, Rotation2d.fromDegrees(174))),
            SHELF_RETRACTED(new Translation2d(MINIMAL_LENGTH_METERS, Rotation2d.fromDegrees(174))),

            MIDDLE(new Translation2d(0.7, Rotation2d.fromDegrees(135))),

            CLOSED(new Translation2d(MINIMAL_LENGTH_METERS - 0.01, Rotation2d.fromDegrees(90))),
            LOCKED(new Translation2d(MAXIMAL_LENGTH_METERS + 0.02, Rotation2d.fromDegrees(86)));

            public final Translation2d setpoint;

            Setpoints(Translation2d setpoint) {
                this.setpoint = setpoint;
            }
        }

        public static final int ANGLE_MOTOR_ID = 21;
        public static final int ANGLE_FOLLOWER_MOTOR_ID = 22;
        public static final int LENGTH_MOTOR_ID = 23;

        public static final int CLOSED_LIMIT_SWITCH_ID = 6;

        public static final int ABS_ANGLE_ENCODER_CHANNEL = 5;

        public static final double MINIMAL_LENGTH_METERS = 0.6175;// m
        public static final double MAXIMAL_LENGTH_METERS = 1.01; // m
        public static final double ROT_TO_METER = 1.0 / 242.5;
        public static final double RPM_TO_METER_PER_SEC = ROT_TO_METER / 60; //link: https://brainly.in/question/3238411

        public static final double ABS_ENCODER_OFFSET_ANGLE_DEG = 0.25167 + 0.5; // NOT IN DEGREES -- IN DUTY CYCLE

        // Angle control
        public static final double kS_ANGLE = -0.048742;
        public static final double kV_ANGLE = 0.020229;
        public static final double kA_ANGLE = 0.0024233;
        public static final double kG_ANGLE = -0.53;

//        static {
//            kG_ANGLE.put(MINIMAL_LENGTH_METERS, 0.50333);
//            kG_ANGLE.put(MAXIMAL_LENGTH_METERS, 0.69438);
//        }

        public static final double kP_ANGLE = 0.061938;
        public static final double kD_ANGLE = 0.015837;

        // Length control
        public static final double kS_LENGTH = 0.070742;
        public static final double kV_LENGTH = 29.296;
        public static final double kG_LENGTH = -0.064553;
        public static final double kP_LENGTH = 4.5124;
        public static final double kD_LENGTH = 3.6247;
        public static final double kMaxLinearVelocity = 0.75;
        public static final double kMaxLinearAcceleration = 0.75;
    }


    public static final class CuberConstants {
        // TODO: update with real values

        public static final int ANGLE_MOTOR_ID = 0;
        public static final int ROLLERS_MOTOR_ID = 0;

        public static final double ABS_ENCODER_OFFSET = 0; // dutyCycle

        public static final int SERVO_CHANNEL = 0;
        public static final int ENCODER_CHANNEL = 0;

        public static final int ULTRASONIC_PING_CHANNEL = 0;
        public static final int ULTRASONIC_ECHO_CHANNEL = 0;
        public static final double ULTRASONIC_THRESHOLD = 10;

        public static final float FWD_SOFT_LIMIT = 170f;
        public static final float REV_SOFT_LIMIT = 5f;

        public static final double ANGLE_CONVERSION_FACTOR = 1.0 / 50.0;

        public static final double VEL_THRESHOLD = 50;
        public static final double POS_THRESHOLD = 1;

        // angle control constants
        public static final double Ks_ANGLE = 0;
        public static final double Kv_ANGLE = 0;
        public static final double Kg_ANGLE = 0;
        public static final double Ka_ANGLE = 0;

        public static final double Kp_ANGLE = 0;
        public static final double Kd_ANGLE = 0;

        // shooter control constants
        public static final double Ks_SHOOTER = 0;
        public static final double Kv_SHOOTER = 0;
        public static final double Ka_SHOOTER = 0;

        public static final double Kp_SHOOTER = 0;
        public static final double Kd_SHOOTER = 0;


        public enum SERVO_ANGLE {
            RETRACTED(0),
            EXTENDED(90);

            public final double angle;

            SERVO_ANGLE(double angle) {
                this.angle = angle;
            }
        }

        public enum CUBER_ANGLE {
            HIGH(165),
            MIDDLE(160),
            LOW(75),
            CLOSED(80),
            INTAKE_GROUND(10),
            INTAKE_SLIDE(120);

            public final int angle;

            CUBER_ANGLE(int angle) {
                this.angle = angle;
            }
        }

        public enum SHOOTER_VELOCITIY {
            HIGH(4000),
            MIDDLE(2000),
            LOW(750),
            INTAKE(-1000);

            public final int velocity;

            SHOOTER_VELOCITIY(int angle) {
                this.velocity = angle;
            }
        }
    }

    public static class LedsConstants {
        public static final int LEDS_PORT_A = 0;
        public static final int LEDS_PORT_B = 1;
        public static final int LENGTH = 133;

        public enum GamePiece {
            Cone(Colors.PURPLE),
            Cube(Colors.ORANGE);

            public Color color;

            GamePiece(Colors color){
                this.color = color.color;
            }
        }
    }
}