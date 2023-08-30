package frc.robot.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.SwerveConstants.Modules.*;

public class Swerve extends SubsystemBase {
    private final SwerveModule[] swerveModules = {
            new SwerveModule(
                    Modules.FL.DRIVE_MOTOR_ID,
                    Modules.FL.SPIN_MOTOR_ID,
                    Modules.FL.DRIVE_MOTOR_REVERSED,
                    Modules.FL.SPIN_MOTOR_REVERSED,
                    Modules.FL.ABS_ENCODER_CHANNEL,
                    Modules.FL.OFFSET_ANGLE),
            new SwerveModule(
                    Modules.FR.DRIVE_MOTOR_ID,
                    Modules.FR.SPIN_MOTOR_ID,
                    Modules.FR.DRIVE_MOTOR_REVERSED,
                    Modules.FR.SPIN_MOTOR_REVERSED,
                    Modules.FR.ABS_ENCODER_CHANNEL,
                    Modules.FR.OFFSET_ANGLE),
            new SwerveModule(
                    Modules.BL.DRIVE_MOTOR_ID,
                    Modules.BL.SPIN_MOTOR_ID,
                    Modules.BL.DRIVE_MOTOR_REVERSED,
                    Modules.BL.SPIN_MOTOR_REVERSED,
                    Modules.BL.ABS_ENCODER_CHANNEL,
                    Modules.BL.OFFSET_ANGLE),
            new SwerveModule(
                    Modules.BR.DRIVE_MOTOR_ID,
                    Modules.BR.SPIN_MOTOR_ID,
                    Modules.BR.DRIVE_MOTOR_REVERSED,
                    Modules.BR.SPIN_MOTOR_REVERSED,
                    Modules.BR.ABS_ENCODER_CHANNEL,
                    Modules.BR.OFFSET_ANGLE)};

    private final AHRS _gyro = new AHRS(SPI.Port.kMXP);

    private final PIDController rampController = new PIDController(Constants.SwerveConstants.RAMP_BALANCE_KP, 0, RAMP_BALANCE_KD);
    private final PIDController thetaTeleopController = new PIDController(kp_Theta, 0, kd_Theta);

    private final Trigger robotBalancedTrigger = new Trigger(() -> Math.abs(getRampAngle()) < 10).debounce(0.35);

    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
            kSwerveKinematics,
            getGyroRotation(),
            getModulesPositions(),
            new Pose2d());

    private final Field2d field = new Field2d();

    public Swerve() {
        resetGyro();

        var swerveTab = Shuffleboard.getTab("Swerve");
        swerveTab.add("FL", swerveModules[FRONT_LEFT]).withWidget(BuiltInWidgets.kGyro)
                .withPosition(4, 0).withSize(4, 4);
        swerveTab.add("FR", swerveModules[FRONT_RIGHT]).withWidget(BuiltInWidgets.kGyro)
                .withPosition(8, 0).withSize(4, 4);
        swerveTab.add("BL", swerveModules[BACK_LEFT]).withWidget(BuiltInWidgets.kGyro)
                .withPosition(4, 4).withSize(4, 4);
        swerveTab.add("BR", swerveModules[BACK_RIGHT]).withWidget(BuiltInWidgets.kGyro)
                .withPosition(8, 4).withSize(4, 4);
        swerveTab.addDouble("SwerveAngle", this::getDegrees).withWidget(BuiltInWidgets.kGyro)
                .withPosition(0, 2).withSize(4, 4);
        swerveTab.add("Field2d", field).withSize(9, 5).withPosition(12, 0);

        RobotContainer.driveTab.addDouble("SwerveAngle", () -> getOdometryRotation().getDegrees())
                .withWidget(BuiltInWidgets.kGyro).withPosition(0, 0).withSize(4, 2);

        thetaTeleopController.enableContinuousInput(-180, 180);
        thetaTeleopController.setTolerance(1.5);
    }

    public void resetGyro() {
        _gyro.reset();
    }

    private double getDegrees() {
        return Math.IEEEremainder(_gyro.getAngle(), 360);
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(getDegrees());
    }

    public Rotation2d getOdometryRotation() {
        return odometry.getEstimatedPosition().getRotation();
    }

    // return the pitch of the robot
    public double getRampAngle() {
        return _gyro.getRoll() - 0.46;
    }

    public void setModulesStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, kPhysicalMaxSpeedMetersPerSecond);
        swerveModules[FRONT_LEFT].setDesiredState(states[FRONT_LEFT]);
        swerveModules[FRONT_RIGHT].setDesiredState(states[FRONT_RIGHT]);
        swerveModules[BACK_LEFT].setDesiredState(states[BACK_LEFT]);
        swerveModules[BACK_RIGHT].setDesiredState(states[BACK_RIGHT]);
    }

    public void setPose2d(Pose2d pose) {
        odometry.resetPosition(getGyroRotation(), getModulesPositions(), pose);
    }

    public Pose2d getPose2d() {
        return odometry.getEstimatedPosition();
    }

    public SwerveModulePosition[] getModulesPositions() {
        return new SwerveModulePosition[]{
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition(),
        };
    }

    public Command resetModulesCommand() {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    swerveModules[FRONT_LEFT].spinTo(0);
                    swerveModules[FRONT_RIGHT].spinTo(0);
                    swerveModules[BACK_LEFT].spinTo(0);
                    swerveModules[BACK_RIGHT].spinTo(0);
                },
                (__) -> {
                    stopModules();
                    resetAngleEncoders();
                },
                swerveModules[FRONT_LEFT].isReset
                        .and(swerveModules[FRONT_RIGHT].isReset)
                        .and(swerveModules[BACK_LEFT].isReset)
                        .and(swerveModules[BACK_RIGHT].isReset),
                this);
    }

    public Command driveSwerveCommand(
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier spinningSpeedSupplier,
            BooleanSupplier fieldOriented) {

        final SlewRateLimiter
                xLimiter = new SlewRateLimiter(kMaxDriveAccelerationUnitsPerSecond),
                yLimiter = new SlewRateLimiter(kMaxDriveAccelerationUnitsPerSecond),
                spinningLimiter = new SlewRateLimiter(kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        return resetModulesCommand().andThen(
                new FunctionalCommand(
                        () -> {
                        },
                        () -> {
                            //create the speeds for x,y and spinning and using a deadBand and Limiter to fix edge cases
                            double xSpeed = xLimiter.calculate(xSpeedSupplier.getAsDouble()) * kMaxDriveSpeed,
                                    ySpeed = yLimiter.calculate(ySpeedSupplier.getAsDouble()) * kMaxDriveSpeed,
                                    spinningSpeed = spinningLimiter.calculate(spinningSpeedSupplier.getAsDouble()) * kMaxTurningSpeed;

                            // create a CassisSpeeds object and apply it the speeds
                            ChassisSpeeds chassisSpeeds = fieldOriented.getAsBoolean() ?
                                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, spinningSpeed, getOdometryRotation()) :
                                    new ChassisSpeeds(xSpeed, ySpeed, spinningSpeed);

                            //use the ChassisSpeedsObject to create an array of SwerveModuleStates
                            SwerveModuleState[] moduleStates = kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

                            //apply the array to the swerve modules of the robot
                            setModulesStates(moduleStates);
                        },
                        (__) -> stopModules(),
                        () -> false,
                        this));
    }

    public Command resetGyroCommand(double angle) {
        return new InstantCommand(() ->
                odometry.resetPosition(
                        getGyroRotation(),
                        getModulesPositions(),
                        new Pose2d(odometry.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(angle))));
    }

    public Command resetGyroCommand() {
        return resetGyroCommand(0);
    }

    // autonomous ramp climbing commands

    public Command driveToRampCommand(boolean forward) {
        final double speed = forward ? 0.45 : -0.45;
        return driveSwerveCommand(() -> speed, () -> 0, () -> 0, () -> true)
                .until(robotBalancedTrigger.negate())
                .andThen(new InstantCommand(this::stopModules));
    }

    public Command balanceRampCommand() {
        return driveSwerveCommand(
                () -> rampController.calculate(getRampAngle(), 0),
                () -> 0,
                () -> 0,
                () -> false);
//          .until(robotBalancedTrigger.debounce(0.2));
    }

    public Command climbCommand(boolean isForward) {
        return driveToRampCommand(isForward)
                .andThen(
                        driveSwerveCommand(() -> 0.375, () -> 0, () -> 0, () -> true).withTimeout(1.315),
                        new WaitCommand(0.15),
                        balanceRampCommand());
    }

    @Override
    public void periodic() {
        odometry.update(getGyroRotation(), getModulesPositions());

//        limelight.updateFromAprilTagPose(odometry::addVisionMeasurement);

        field.setRobotPose(odometry.getEstimatedPosition());
        SmartDashboard.putData(field);
    }

    private void resetAngleEncoders() {
        swerveModules[FRONT_LEFT].resetEncoders();
        swerveModules[FRONT_RIGHT].resetEncoders();
        swerveModules[BACK_LEFT].resetEncoders();
        swerveModules[BACK_RIGHT].resetEncoders();
    }

    private void stopModules() {
        swerveModules[FRONT_LEFT].stopModule();
        swerveModules[FRONT_RIGHT].stopModule();
        swerveModules[BACK_LEFT].stopModule();
        swerveModules[BACK_RIGHT].stopModule();
    }
}
