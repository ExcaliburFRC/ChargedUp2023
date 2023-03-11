package frc.robot.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.utility.Limelight;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.math.MathUtil.applyDeadband;
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

  private final PIDController rampController = new PIDController(Constants.SwerveConstants.RAMP_BALANCE_KP, 0, 0);
  private final PIDController thetaTeleopController = new PIDController(kp_Theta_TELEOP, 0, 0);
  private final PIDController xController = new PIDController(0, 0, 0);
  private final PIDController yController = new PIDController(0, 0, 0);

  private final AtomicInteger lastJoystickAngle = new AtomicInteger(0);
  private final Trigger robotBalancedTrigger = new Trigger(() -> Math.abs(getRampAngle()) < 3);

  //    private final Limelight limelight = new Limelight();
  private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
        kSwerveKinematics,
        getGyroRotation(),
        new SwerveModulePosition[]{
              swerveModules[FRONT_LEFT].getPosition(),
              swerveModules[FRONT_RIGHT].getPosition(),
              swerveModules[BACK_LEFT].getPosition(),
              swerveModules[BACK_RIGHT].getPosition()},
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

  private final Field2d field = new Field2d();

  public Swerve() {
    resetGyro();

    thetaTeleopController.enableContinuousInput(-180, 180);
    thetaTeleopController.setTolerance(1.5);

    var swerveTab = Shuffleboard.getTab("Swerve");
    swerveTab.add("FL", swerveModules[FRONT_LEFT]).withWidget(BuiltInWidgets.kGyro)
          .withPosition(0, 0).withSize(2, 2);
    swerveTab.add("FR", swerveModules[FRONT_RIGHT]).withWidget(BuiltInWidgets.kGyro)
          .withPosition(2, 0).withSize(2, 2);
    swerveTab.add("BL", swerveModules[BACK_LEFT]).withWidget(BuiltInWidgets.kGyro)
          .withPosition(0, 2).withSize(2, 2);
    swerveTab.add("BR", swerveModules[BACK_RIGHT]).withWidget(BuiltInWidgets.kGyro)
          .withPosition(2, 2).withSize(2, 2);
    swerveTab.addDouble("SwerveAngle", () -> getRotation().getDegrees()).withWidget(BuiltInWidgets.kGyro)
          .withPosition(1, 4).withSize(2, 2);
    swerveTab.add("Field2d", field).withSize(9, 5).withPosition(4, 0);

    odometry.resetPosition(
          getGyroRotation(),
          new SwerveModulePosition[]{
                swerveModules[FRONT_LEFT].getPosition(),
                swerveModules[FRONT_RIGHT].getPosition(),
                swerveModules[BACK_LEFT].getPosition(),
                swerveModules[BACK_RIGHT].getPosition()},
          new Pose2d(8.28, 4, new Rotation2d())
    );
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

  public Rotation2d getRotation() {
    return odometry.getEstimatedPosition().getRotation();
  }


  // return the pitch of the robot
  //TODO: check if works
  private double getRampAngle() {
    double pitch = _gyro.getPitch();
//        pitch *= -1;
//        if (pitch < 0) pitch += 360;
    return pitch * 100;
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
            resetEncoders();
            new PrintCommand("modules are reset").schedule();
          },
          () -> swerveModules[FRONT_LEFT].isReset
                .and(swerveModules[FRONT_RIGHT].isReset)
                .and(swerveModules[BACK_LEFT].isReset)
                .and(swerveModules[BACK_RIGHT].isReset)
                .getAsBoolean(),
          this);
  }

  // returns a command that enables the driver to control the swerve at each method
  public Command dualDriveSwerveCommand(
        DoubleSupplier xSpeedSupplier,
        DoubleSupplier ySpeedSupplier,
        DoubleSupplier xAngle,
        DoubleSupplier yAngle,
        BooleanSupplier fieldOriented,
        BooleanSupplier withAngle) {
    return new RepeatCommand(
          driveSwerveCommand(xSpeedSupplier, ySpeedSupplier, xAngle, fieldOriented)
                .until(withAngle)
                .andThen(driveSwerveWithAngleCommand(xSpeedSupplier, ySpeedSupplier, () -> -xAngle.getAsDouble(), () -> yAngle.getAsDouble(), fieldOriented)
                      .until(() -> !withAngle.getAsBoolean())));
  }

  // turning speed based swerve drive
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
                }, //resetModulesCommand().schedule(),
                () -> {
                  //create the speeds for x,y and spinning and using a deadBand and Limiter to fix edge cases
                  double xSpeed = xLimiter.calculate(applyDeadband(xSpeedSupplier.getAsDouble(), kDeadband)) * kMaxDriveSpeed,
                        ySpeed = yLimiter.calculate(applyDeadband(ySpeedSupplier.getAsDouble(), kDeadband)) * kMaxDriveSpeed,
                        spinningSpeed = spinningLimiter.calculate(applyDeadband(spinningSpeedSupplier.getAsDouble(), kDeadband)) * kMaxDriveTurningSpeed;

                  // create a CassisSpeeds object and apply it the speeds
                  ChassisSpeeds chassisSpeeds = fieldOriented.getAsBoolean() ?
                        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, spinningSpeed, getRotation()) :
                        new ChassisSpeeds(xSpeed, ySpeed, spinningSpeed);

                  //use the ChassisSpeedsObject to create an array of SwerveModuleStates
                  SwerveModuleState[] moduleStates = kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

                  //apply the array to the swerve modules of the robot
                  setModulesStates(moduleStates);
                },
                (__) -> {
                  stopModules();
                  resetEncoders();
                },
                () -> false,
                this));
  }

  public Command tankDriveCommand(DoubleSupplier speed, DoubleSupplier turn, boolean fielOriented) {
    return driveSwerveCommand(() -> 0, speed, turn, () -> fielOriented);
  }

  // angle based swerve drive
  public Command driveSwerveWithAngleCommand(
        DoubleSupplier xSpeed,
        DoubleSupplier ySpeed,
        DoubleSupplier xAngle,
        DoubleSupplier yAngle,
        BooleanSupplier fieldOriented) {
    return new InstantCommand(() -> lastJoystickAngle.set((int) getDegrees()))
          .andThen(new ParallelCommandGroup(
                new RunCommand(() -> updateJoystickAngle(xAngle.getAsDouble(), yAngle.getAsDouble())),
                driveSwerveCommand(xSpeed, ySpeed, () -> -thetaTeleopController.calculate(getDegrees(), lastJoystickAngle.get()), fieldOriented))
          );
  }

  // returns the joystick angle in 0 to 360 (right is 0, up is 90)
  //TODO: fix and make simple
  private double calculateJoystickAngle(double x, double y) {
    double a = -Math.toDegrees(Math.atan(y / x));
    if (x < 0) a += 180;
    if (a < 0) a += 360;
    a -= 90;
    if (a < 0) a += 360;
    if (a > 180) a -= 360;
    return -a;
  }

  //update the angle of the joystick
  private void updateJoystickAngle(double x, double y) {
    if (Math.abs(x) + Math.abs(y) > 0.5) {
      lastJoystickAngle.set((int) calculateJoystickAngle(x, y));
    }
  }

  public Command resetJoystickAngle() {
    return new InstantCommand(() -> lastJoystickAngle.set(0));
  }

  public Command resetOdometryCommand(Pose2d newPose) {
    return new InstantCommand(() -> odometry.resetPosition(
          getGyroRotation(),
          new SwerveModulePosition[]{
                swerveModules[FRONT_LEFT].getPosition(),
                swerveModules[FRONT_RIGHT].getPosition(),
                swerveModules[BACK_LEFT].getPosition(),
                swerveModules[BACK_RIGHT].getPosition()
          },
          newPose)
    );
  }

  public Command resetGyroCommand(double angle) {
    return new InstantCommand(() ->
          odometry.resetPosition(
                getGyroRotation(),
                new SwerveModulePosition[]{
                      swerveModules[FRONT_LEFT].getPosition(),
                      swerveModules[FRONT_RIGHT].getPosition(),
                      swerveModules[BACK_LEFT].getPosition(),
                      swerveModules[BACK_RIGHT].getPosition()},
                new Pose2d(odometry.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(angle))));
  }

  public Command resetGyroCommand() {
    return resetGyroCommand(0);
  }


  public Command autoMotionCommand(boolean fieldOriented, Pose2d... poses) {
    double xyTolerance = 0, angleTolerance = 0;
    AtomicInteger currentPoseIndex = new AtomicInteger();
    return Commands.repeatingSequence(
          driveSwerveCommand(
                () -> xController.calculate(
                      odometry.getEstimatedPosition().getX(),
                      poses[currentPoseIndex.get()].getX()),

                () -> yController.calculate(
                      odometry.getEstimatedPosition().getY(),
                      poses[currentPoseIndex.get()].getY()),

                () -> thetaTeleopController.calculate(
                      odometry.getEstimatedPosition().getRotation().getDegrees(),
                      poses[currentPoseIndex.get()].getRotation().getDegrees()),

                () -> fieldOriented)
                .until(() -> poseInTolerance(odometry.getEstimatedPosition(), poses[currentPoseIndex.get()])),
          new InstantCommand(currentPoseIndex::getAndIncrement)
    );
  }

  private boolean poseInTolerance(Pose2d measurement, Pose2d setPoint) {
    double xTolerance = 0,
          yTolerance = 0,
          thetaTolerance = 0,
          xAbsError = Math.abs(setPoint.getX() - measurement.getX()),
          yAbsError = Math.abs(setPoint.getY() - measurement.getY()),
          setPointDegrees = setPoint.getRotation().getDegrees(),
          measuredDegrees = measurement.getRotation().getDegrees(),
          thetaAbsError = setPointDegrees - measuredDegrees;
    if (thetaAbsError < 0) thetaAbsError = thetaAbsError += 360;
    if (thetaAbsError > 180) thetaAbsError -= 180;
    thetaAbsError = Math.abs(thetaAbsError);
    return xAbsError < xTolerance && yAbsError < yTolerance && thetaAbsError < thetaTolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("gyro angle", _gyro);

    odometry.update(
          getGyroRotation(),
          new SwerveModulePosition[]{
                swerveModules[FRONT_LEFT].getPosition(),
                swerveModules[FRONT_RIGHT].getPosition(),
                swerveModules[BACK_LEFT].getPosition(),
                swerveModules[BACK_RIGHT].getPosition()
          });

//        limelight.updateFromAprilTagPose(odometry::addVisionMeasurement);

    field.setRobotPose(odometry.getEstimatedPosition());
    SmartDashboard.putData(field);
  }

//    @Override
//    public void initSendable(SendableBuilder builder) {
//        builder.setSmartDashboardType("Gyro");
//        builder.addDoubleProperty("Value", () -> odometry.getEstimatedPosition().getRotation().getDegrees(), null);
//        builder.addDoubleProperty("ramp angle", this::getRampAngle, null);
//    }

  private void setModulesStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kPhysicalMaxSpeedMetersPerSecond);
    swerveModules[FRONT_LEFT].setDesiredState(states[FRONT_LEFT]);
    swerveModules[FRONT_RIGHT].setDesiredState(states[FRONT_RIGHT]);
    swerveModules[BACK_LEFT].setDesiredState(states[BACK_LEFT]);
    swerveModules[BACK_RIGHT].setDesiredState(states[BACK_RIGHT]);
  }

  private void resetEncoders() {
    swerveModules[FRONT_LEFT].resetEncoders();
    swerveModules[FRONT_RIGHT].resetEncoders();
    swerveModules[BACK_LEFT].resetEncoders();
    swerveModules[BACK_RIGHT].resetEncoders();
  }

  public Command driveToRampCommand(boolean forward) {
    final double speed = forward ? 0.2 : -0.2;
    return tankDriveCommand(() -> speed, () -> 0, false)
          .until(robotBalancedTrigger.negate())
          .andThen(new InstantCommand(this::stopModules));
  }

  public Command balanceRampCommand() {
    return driveSwerveCommand(
          () -> 0,
          () -> rampController.calculate(getRampAngle(), 0),
          () -> 0,
          () -> true)
          .until(robotBalancedTrigger.debounce(0.2));
  }

  public Command climbCommand(boolean isForward) {
    return driveToRampCommand(isForward).andThen(balanceRampCommand());
  }

  private void stopModules() {
    swerveModules[FRONT_LEFT].stop();
    swerveModules[FRONT_RIGHT].stop();
    swerveModules[BACK_LEFT].stop();
    swerveModules[BACK_RIGHT].stop();
  }
}
