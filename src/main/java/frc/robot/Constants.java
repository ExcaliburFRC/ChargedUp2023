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
        public static final int BEAMBREAK_PORT = 2;
        public static final int RIGHT_ROLLER_MOTOR_ID = 31;
        public static final int LEFT_ROLLER_MOTOR_PORT = 8;
    }

    public static final class SwerveConstants {
        public enum Modules {
            // drive ID, spin ID, abs encoder channel, offset angle, drive reversed, angle reversed
            FL(18, 17, 5, 0.093 + 0.25, false, false),
            FR(12, 11, 9, 0.372 + 0.25, false, false),
            BL(16, 15, 4, 0.05, false, false),
            BR(14, 13, 8, 0.834, false, false);

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
        public static final double kSpeedPercantageLimit = 100; // %
        public static final double kMaxDriveSpeed = kPhysicalMaxSpeedMetersPerSecond / 100 * kSpeedPercantageLimit; // m/s
        public static final double kMaxDriveTurningSpeed = kPhysicalMaxAngularSpeedRadiansPerSecond / 100 * kSpeedPercantageLimit;// rad/s
        public static final double kMaxTurningAcceleration = PI / 100 * kSpeedPercantageLimit; // rad/s^2

        //unclear values
        public static final double kMaxDriveAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;

        // autonomous constants
        public static final double kp_ANGLE = 0.00585;
        public static final double kd_ANGLE = 0;
        public static final double kp_TRANSLATION = 0;
        public static final double kd_TRANSLATION = 0;
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
            LOW(new Translation2d(0.7, Rotation2d.fromDegrees(130))),//
            MID(new Translation2d(0.72, Rotation2d.fromDegrees(172))),
            HIGH_CHECKPOINT(new Translation2d(MAXIMAL_LENGTH_METERS, Rotation2d.fromDegrees(130))),
            HIGH(new Translation2d(MAXIMAL_LENGTH_METERS, Rotation2d.fromDegrees(192))),
            SHELF_EXTENDED(new Translation2d(0.75, Rotation2d.fromDegrees(174))),
            SHELF_RETRACTED(new Translation2d(MINIMAL_LENGTH_METERS, Rotation2d.fromDegrees(174))),

            MIDDLE(new Translation2d(0.7, Rotation2d.fromDegrees(135))),
            LOCKED(new Translation2d(LOCKED_LENGTH_METERS, Rotation2d.fromDegrees(88))),

            LEANED(new Translation2d(MINIMAL_LENGTH_METERS, Rotation2d.fromDegrees(90)));

            public final Translation2d setpoint;

            Setpoints(Translation2d setpoint) {
                this.setpoint = setpoint;
            }
        }

        public static final int ANGLE_MOTOR_ID = 21;
        public static final int ANGLE_FOLLOWER_MOTOR_ID = 22;
        public static final int LENGTH_MOTOR_ID = 23;

        public static final int CLOSED_LIMIT_SWITCH_ID = 6;

        public static final int ABS_ANGLE_ENCODER_PORT = 7;

        public static final double MINIMAL_LENGTH_METERS = 0.6175;// m
        public static final double MAXIMAL_LENGTH_METERS = 1.08; // m
        public static final double LOCKED_LENGTH_METERS = 1.01; // m
        public static final double ROT_TO_METER = 1.0 / 242.5;
        public static final double RPM_TO_METER_PER_SEC = ROT_TO_METER / 60; //https://brainly.in/question/3238411

        public static final double ABS_ENCODER_OFFSET_ANGLE_DEG = 0.4234 + 0.5; // NOT IN DEGREES -- IN DUTY CYCLE

        // Angle control
        public static final double kS_ANGLE = -0.048742;
        public static final double kV_ANGLE = 0.020229;
        public static final double kA_ANGLE = 0.0024233;
        public static final double kG_ANGLE = -0.56;

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
        public static final int ANGLE_MOTOR_ID = 41;
        public static final int ROLLERS_MOTOR_ID = 42;

        public static final double ABS_ENCODER_OFFSET = 0.475; // dutyCycle

        public static final int SERVO_CHANNEL = 9;
        public static final int ENCODER_CHANNEL = 3;

        public static final double COLOR_DISTANCE_THRESHOLD = 100;
        public static final double ROBOT_ANGLE_THRESHOLD = 10; // degrees

        public static final int MAX_ANGLE_DEGREES = 137; // deg

        public static final double FWD_SOFT_LIMIT = MAX_ANGLE_DEGREES;
        public static final double REV_SOFT_LIMIT = 0;

        public static final double ANGLE_CONVERSION_FACTOR = (1.0 / 50.0) * (35.0 / 73.0); //104.2857

        public static final double VEL_THRESHOLD = SHOOTER_VELOCITIY.HIGH.velocity * 0.05;
        public static final double POS_THRESHOLD = 3;


        // angle control constants
        public static final double Ks_ANGLE = 0.14942;
        public static final double Kv_ANGLE = 0.063608;
        public static final double Kg_ANGLE = 0.43944;
        public static final double Ka_ANGLE = 0.001574;

        public static final double Kp_ANGLE = 0.091365 / 1.3;
        public static final double Kd_ANGLE = 0; //0.034286

        // shooter control constants
        public static final double Ks_SHOOTER = 0.15534;
        public static final double Kv_SHOOTER = 0.13081;
        public static final double Ka_SHOOTER = 0.0067467;

        public static final double Kp_SHOOTER = 0.00020361;
        public static final double Kd_SHOOTER = 0;


        public enum SERVO_ANGLE {
            RETRACTED(0),
            EXTENDED(45);

            public final double angle;

            SERVO_ANGLE(double angle) {
                this.angle = angle;
            }
        }

        public enum CUBER_ANGLE {
            HIGH(MAX_ANGLE_DEGREES),
            MIDDLE(128),
            LOW(92),
            CANNON(100),
            IDLE(110),
            INTAKE_GROUND(0),
            INTAKE_SLIDE(HIGH.angle);

            public final int angle;

            CUBER_ANGLE(int angle) {
                this.angle = angle;
            }
        }

        public enum SHOOTER_VELOCITIY {
            HIGH(1500),
            MIDDLE(1000),
            LOW(800),
            CANNON(2500),
            IDLE(-300),
            INTAKE(-1200);

            public final int velocity;

            SHOOTER_VELOCITIY(int angle) {
                this.velocity = angle;
            }
        }
    }

    public static class LedsConstants {
        public static final int LEDS_PORT = 0; // pwm
        public static final int LENGTH = 139;

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