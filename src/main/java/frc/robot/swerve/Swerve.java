package frc.robot.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.HashMap;
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

  private final PIDController rampController = new PIDController(Constants.SwerveConstants.RAMP_BALANCE_KP, 0, RAMP_BALANCE_KD);
  private final PIDController thetaTeleopController = new PIDController(kp_Theta, 0, kd_Theta);

  private final AtomicInteger lastJoystickAngle = new AtomicInteger(0);
  private final Trigger robotBalancedTrigger = new Trigger(() -> Math.abs(getRampAngle()) < 10).debounce(0.35);

  private final SlewRateLimiter angleRateLimiter = new SlewRateLimiter(0.15);

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

  private final SwerveAutoBuilder SwerveAutoBuilder = new SwerveAutoBuilder(
        () -> new Pose2d(getPose2d().getTranslation(), Rotation2d.fromDegrees(-getPose2d().getRotation().getDegrees())),
        this::setPose2d,
        Constants.SwerveConstants.kSwerveKinematics,
        new PIDConstants(0,0,0),
        new PIDConstants(0,0,0),
        this::setModulesStates,
        new HashMap<>(),
        false,
        this
  );

  private final Field2d field = new Field2d();

  public Swerve() {
    resetGyro();

    thetaTeleopController.enableContinuousInput(-180, 180);
    thetaTeleopController.setTolerance(1.5);

    var swerveTab = Shuffleboard.getTab("Swerve");
    swerveTab.add("FL", swerveModules[FRONT_LEFT]).withWidget(BuiltInWidgets.kGyro)
          .withPosition(4, 0).withSize(4, 4);
    swerveTab.add("FR", swerveModules[FRONT_RIGHT]).withWidget(BuiltInWidgets.kGyro)
          .withPosition(8, 0).withSize(4, 4);
    swerveTab.add("BL", swerveModules[BACK_LEFT]).withWidget(BuiltInWidgets.kGyro)
          .withPosition(4, 4).withSize(4, 4);
    swerveTab.add("BR", swerveModules[BACK_RIGHT]).withWidget(BuiltInWidgets.kGyro)
          .withPosition(8, 4).withSize(4, 4);
    swerveTab.addDouble("SwerveAngle", () -> getDegrees()).withWidget(BuiltInWidgets.kGyro)
          .withPosition(0, 2).withSize(4, 4);
    swerveTab.add("Field2d", field).withSize(9, 5).withPosition(12, 0);

    swerveTab.addDouble("rampAngle", () -> getRampAngle()).withSize(2, 2);
    swerveTab.addDouble("pid ramp", () -> rampController.calculate(getRampAngle(), 0)).withSize(2, 2);
    swerveTab.addBoolean("robot balanced", robotBalancedTrigger::getAsBoolean);

    RobotContainer.driveTab.addDouble("SwerveAngle", () -> getOdometryRotation().getDegrees())
          .withWidget(BuiltInWidgets.kGyro).withPosition(0, 0).withSize(4, 2);

    odometry.resetPosition(
          getGyroRotation(),
          getModulePositions(),
          new Pose2d(0, 0, new Rotation2d(0)));

    thetaTeleopController.setTolerance(1);
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
  //TODO: check if works
  public double getRampAngle() {
    double roll = _gyro.getRoll();
    roll -= 0.46;
    return roll;
  }

  public void setPose2d(Pose2d pose) {
    odometry.resetPosition(getGyroRotation(), getModulePositions(), pose);
  }

  public Pose2d getPose2d(){
    return odometry.getEstimatedPosition();
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

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{
          swerveModules[0].getPosition(),
          swerveModules[1].getPosition(),
          swerveModules[2].getPosition(),
          swerveModules[3].getPosition(),
    };
  }

  // turning speed based swerve drive
  public Command driveSwerveCommand(
        DoubleSupplier xSpeedSupplier,
        DoubleSupplier ySpeedSupplier,
        DoubleSupplier spinningSpeedSupplier,
        BooleanSupplier fieldOriented,
        BooleanSupplier slowMode) {

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
                  double xSpeed = xLimiter.calculate(xSpeedSupplier.getAsDouble()) * kMaxDriveSpeed,
                        ySpeed = yLimiter.calculate(ySpeedSupplier.getAsDouble()) * kMaxDriveSpeed,
                        spinningSpeed = spinningLimiter.calculate(slowMode.getAsBoolean() ? spinningSpeedSupplier.getAsDouble() / 5 : spinningSpeedSupplier.getAsDouble()) * kMaxDriveTurningSpeed;

                  // create a CassisSpeeds object and apply it the speeds
                  ChassisSpeeds chassisSpeeds = fieldOriented.getAsBoolean() ?
                        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, spinningSpeed, getOdometryRotation()) :
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

  public Command driveSwerveCommand(
        DoubleSupplier xSpeedSupplier,
        DoubleSupplier ySpeedSupplier,
        DoubleSupplier spinningSpeedSupplier,
        BooleanSupplier fieldOriented){
    return driveSwerveCommand(xSpeedSupplier, ySpeedSupplier, spinningSpeedSupplier, fieldOriented, ()-> false);
  }

  public Command tankDriveCommand(DoubleSupplier speed, DoubleSupplier turn, boolean fieldOriented) {
    return driveSwerveCommand(speed, () -> 0, turn, () -> fieldOriented);
  }

  // angle based swerve drive
  public Command driveSwerveWithAngleCommand(
        DoubleSupplier xSpeed,
        DoubleSupplier ySpeed,
        DoubleSupplier angle,
//        DoubleSupplier xAngle,
//        DoubleSupplier yAngle,
        BooleanSupplier fieldOriented) {
//    return new InstantCommand(() -> lastJoystickAngle.set((int) getRotation().getDegrees()))
          return new ParallelCommandGroup(
//                new RunCommand(() -> updateJoystickAngle(xAngle.getAsDouble(), yAngle.getAsDouble())),
                driveSwerveCommand(xSpeed, ySpeed, () -> thetaTeleopController.calculate(getOdometryRotation().getDegrees(), angle.getAsDouble()), fieldOriented)
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

  public Command turnToAngleCommand(double setpoint) {
    return driveSwerveCommand(
            () -> 0,
            () -> 0,
            () -> thetaTeleopController.calculate(getOdometryRotation().getDegrees(), setpoint), () -> false)
          .until(new Trigger(thetaTeleopController::atSetpoint).debounce(0.15));
  }

  @Override
  public void periodic() {
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

  public Command followTrajectoryCommand(PathPlannerTrajectory traj){
    return SwerveAutoBuilder.fullAuto(traj);
  }

  public void setModulesStates(SwerveModuleState[] states) {
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
                    tankDriveCommand(()-> 0.375, ()-> 0, true)
                            .withTimeout(1.315),
                    new WaitCommand(0.25),
                    balanceRampCommand());
  }

  private void stopModules() {
    swerveModules[FRONT_LEFT].stop();
    swerveModules[FRONT_RIGHT].stop();
    swerveModules[BACK_LEFT].stop();
    swerveModules[BACK_RIGHT].stop();
  }
}
