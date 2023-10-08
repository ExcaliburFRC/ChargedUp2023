package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
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
import java.util.function.Consumer;
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

    private boolean hasStraighten = false;

    private final AHRS _gyro = new AHRS(SPI.Port.kMXP);

    private final PIDController rampController = new PIDController(Constants.SwerveConstants.RAMP_BALANCE_KP, 0, RAMP_BALANCE_KD);

    private final PIDController angleTeleopController = new PIDController(kp_ANGLE, 0, kd_ANGLE);
    private final PIDController distanceTeleopController = new PIDController(kp_TRANSLATION, 0, kd_TRANSLATION);

    public final Trigger robotBalancedTrigger = new Trigger(() -> Math.abs(getRobotPitch()) < 10).debounce(0.35);

    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
            kSwerveKinematics,
            getGyroRotation2d(),
            getModulesPositions(),
            new Pose2d());

    private final Field2d field = new Field2d();

    public Swerve() {
        resetGyroHardware();

        odometry.resetPosition(getGyroRotation2d(), getModulesPositions(), new Pose2d(0, 0, new Rotation2d()));
        resetOdometryAngleCommand();

        angleTeleopController.enableContinuousInput(0, 360);
        angleTeleopController.setTolerance(1);

        initShuffleboardData();
    }

    // gyro getters and setters
    public void resetGyroHardware() {
        _gyro.reset();
    }

    private double getGyroDegrees() {
        return Math.IEEEremainder(_gyro.getAngle(), 360);
    }

    public double getRobotPitch() {
        return _gyro.getRoll() - 0.46;
    }

    // odometry getters and setters
    private void setPose2d(Pose2d pose) {
        odometry.resetPosition(getGyroRotation2d(), getModulesPositions(), pose);
    }

    public Pose2d getPose2d() {
        return odometry.getEstimatedPosition();
    }

    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getGyroDegrees());
    }

    public Rotation2d getOdometryRotation2d() {
        return odometry.getEstimatedPosition().getRotation();
    }

    public Command setOdometryPositionCommand(Pose2d pose) {
        return new InstantCommand(() -> setPose2d(pose));
    }

    public Command setOdometryAngleCommand(double angle) {
        return setOdometryPositionCommand(
                new Pose2d(odometry.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(angle)))
                .ignoringDisable(true);
    }

    public Command resetOdometryAngleCommand() {
        return setOdometryAngleCommand(0);
    }

    // drive commands
    public Command driveSwerveCommand(
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier spinningSpeedSupplier,
            BooleanSupplier fieldOriented,
            DoubleSupplier decelerator,
            DoubleSupplier turnToAngle) {

        final SlewRateLimiter
                xLimiter = new SlewRateLimiter(kMaxDriveAccelerationUnitsPerSecond),
                yLimiter = new SlewRateLimiter(kMaxDriveAccelerationUnitsPerSecond),
                spinningLimiter = new SlewRateLimiter(kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        return new ConditionalCommand(new InstantCommand(), straightenModulesCommand(), () -> hasStraighten)
                .andThen(
                        this.runEnd(
                                () -> {
                                    double spinning = turnToAngle.getAsDouble() == -1 ? spinningSpeedSupplier.getAsDouble() : getAngleDC(turnToAngle.getAsDouble());

                                    //create the speeds for x,y and spin
                                    double xSpeed = xLimiter.calculate(xSpeedSupplier.getAsDouble()) * kMaxDriveSpeed * decelerator.getAsDouble(),
                                            ySpeed = yLimiter.calculate(ySpeedSupplier.getAsDouble()) * kMaxDriveSpeed * decelerator.getAsDouble(),
                                            spinningSpeed = spinningLimiter.calculate(spinning) * kMaxTurningSpeed * decelerator.getAsDouble();

                                    // **all credits to the decelerator idea are for Ofir from trigon #5990 (ohfear_ on discord)**

                                    // create a CassisSpeeds object and apply it the speeds
                                    ChassisSpeeds chassisSpeeds = fieldOriented.getAsBoolean() ?
                                            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, spinningSpeed, getOdometryRotation2d()) :
                                            new ChassisSpeeds(xSpeed, ySpeed, spinningSpeed);

                                    //use the ChassisSpeedsObject to create an array of SwerveModuleStates
                                    SwerveModuleState[] moduleStates = kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

                                    //apply the array to the swerve modules of the robot
                                    setModulesStates(moduleStates);
                                },
                                this::stopModules
                        ));
    }

    public Command driveSwerveCommand(
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier spinningSpeedSupplier,
            BooleanSupplier fieldOriented) {
        return driveSwerveCommand(xSpeedSupplier, ySpeedSupplier, spinningSpeedSupplier, fieldOriented, () -> 1, () -> -1);
    }

    public Command tankDriveCommand(DoubleSupplier speed, DoubleSupplier turn, boolean fieldOriented) {
        return driveSwerveCommand(speed, () -> 0, turn, () -> fieldOriented);
    }

    /**
     * an enhanced push command that allows our robot to push other robots, with much greater force,
     * this is achieved by using a similar method to the ABS system in modern cars.
     * where the car's brakes lock up & release very rapidly during an emergency break
     * to prevent slipping, (or motor stalling in our case)
     * @return the command
     */
    public Command enhancedPushCommand(){
        return Commands.repeatingSequence(
                this.run(this::stopModules).withTimeout(ENHANCED_PUSH_FREQUENCY),
                new InstantCommand(()-> getDefaultCommand().withTimeout(ENHANCED_PUSH_FREQUENCY).schedule())
        );
    }

    public Command straightenModulesCommand() {
        return new FunctionalCommand(
                () -> hasStraighten = true,
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

    public Command turnToAngleCommand(double setpoint) {
        return new InstantCommand(() -> angleTeleopController.setSetpoint(setpoint)).andThen(
                driveSwerveCommand(
                        () -> 0, () -> 0,
                        () -> angleTeleopController.calculate(getOdometryRotation2d().getDegrees(), setpoint),
                        () -> false).until(new Trigger(angleTeleopController::atSetpoint).debounce(0.1)));
    }

    public double getAngleDC(double angle) {
        return angleTeleopController.calculate(getOdometryRotation2d().getDegrees(), angle);
    }

    public double getAngleDC() {
        return getAngleDC(0);
    }

    // autonomous ramp climbing commands
    public Command driveToRampCommand(boolean forward) {
        final double speed = forward ? 0.45 : -0.45;
        return driveSwerveCommand(() -> speed, () -> 0, () -> 0, () -> true)
                .until(robotBalancedTrigger.negate())
                .andThen(new InstantCommand(this::stopModules));
    }

    public Command balanceRampCommand() {
//        return new RunCommand(()-> System.out.println(Math.min(rampController.calculate(getRobotPitch(), 0), 0.15)));
        return driveSwerveCommand(
                () -> MathUtil.clamp(rampController.calculate(getRobotPitch(), 0), -0.35, 0.35),
                () -> 0,
                () -> 0,
                () -> true,
                () -> 0.9,
                () -> -1)
                .until(robotBalancedTrigger.debounce(0.2)).andThen(new InstantCommand(this::stopModules, this));
    }

    /**
     * runs the climb sequence <b> **Warning - if ran while robot already on ramp
     * it will jump off aggressively </b>
     *
     * @param isForward is the ramp in front of the robot in field relative
     * @return the command
     */
    public Command climbCommand(boolean isForward, double offset) {
        return driveToRampCommand(isForward).andThen(
                driveSwerveCommand(() -> (isForward ? 1 : -1) * (0.4 + offset), () -> 0, () -> 0, () -> true).withTimeout(1.2),
                new WaitCommand(0.15),
                balanceRampCommand());
    }

    // other methods
    private void resetAngleEncoders() {
        foreachModule(SwerveModule::resetEncoders);
    }

    private void stopModules() {
        foreachModule(SwerveModule::stopModule);
    }

    public void setModulesStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, kPhysicalMaxSpeedMetersPerSecond);
        swerveModules[FRONT_LEFT].setDesiredState(states[FRONT_LEFT]);
        swerveModules[FRONT_RIGHT].setDesiredState(states[FRONT_RIGHT]);
        swerveModules[BACK_LEFT].setDesiredState(states[BACK_LEFT]);
        swerveModules[BACK_RIGHT].setDesiredState(states[BACK_RIGHT]);
    }

    public SwerveModulePosition[] getModulesPositions() {
        return new SwerveModulePosition[]{
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition(),
        };
    }

    private void foreachModule(Consumer<SwerveModule> module) {
        for (int i = 0; i < swerveModules.length; i++) {
            module.accept(swerveModules[i]);
        }
    }

    public Command toggleIdleModeCommand() {
        return new StartEndCommand(
                () -> foreachModule(SwerveModule::setIdleModeCoast),
                () -> foreachModule(SwerveModule::setIdleModebreak))
                .ignoringDisable(true);
    }

    @Override
    public void periodic() {
        odometry.update(getGyroRotation2d(), getModulesPositions());

//        limelight.updateFromAprilTagPose(odometry::addVisionMeasurement);

        field.setRobotPose(odometry.getEstimatedPosition());
        SmartDashboard.putData(field);
    }

    private static double getFwd(boolean isForward) {
        return isForward ? 0 : 180;
    }

    private void initShuffleboardData() {
        var swerveTab = Shuffleboard.getTab("Swerve");
        swerveTab.add("FL", swerveModules[FRONT_LEFT]).withWidget(BuiltInWidgets.kGyro)
                .withPosition(4, 0).withSize(4, 4);
        swerveTab.add("FR", swerveModules[FRONT_RIGHT]).withWidget(BuiltInWidgets.kGyro)
                .withPosition(8, 0).withSize(4, 4);
        swerveTab.add("BL", swerveModules[BACK_LEFT]).withWidget(BuiltInWidgets.kGyro)
                .withPosition(4, 4).withSize(4, 4);
        swerveTab.add("BR", swerveModules[BACK_RIGHT]).withWidget(BuiltInWidgets.kGyro)
                .withPosition(8, 4).withSize(4, 4);
        swerveTab.addDouble("SwerveAngle", () -> getOdometryRotation2d().getDegrees()).withWidget(BuiltInWidgets.kGyro)
                .withPosition(0, 2).withSize(4, 4);
        swerveTab.add("Field2d", field).withSize(9, 5).withPosition(12, 0);
        swerveTab.addDouble("robotPitch", this::getRobotPitch);

        RobotContainer.driveTab.addDouble("SwerveAngle", () -> getOdometryRotation2d().getDegrees())
                .withWidget(BuiltInWidgets.kGyro).withPosition(0, 0).withSize(4, 2);

    }
}
