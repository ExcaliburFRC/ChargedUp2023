package frc.robot.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.utiliy.Limelight;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static frc.robot.Constants.SwerveConstants.Modules.*;
import static frc.robot.Constants.SwerveConstants.*;

public class Swerve extends SubsystemBase {
	//create swerve modules
	private final SwerveModule[] swerveModules = {
			new SwerveModule(
					kDriveMotorId[FRONT_LEFT],
					kSpinningMotorId[FRONT_LEFT],
					kDriveMotorReversed[FRONT_LEFT],
					kSpinningMotorReversed[FRONT_LEFT],
					kAbsEncoderChannel[FRONT_LEFT],
					kOffsetAngle[FRONT_LEFT]),
			new SwerveModule(
					kDriveMotorId[FRONT_RIGHT],
					kSpinningMotorId[FRONT_RIGHT],
					kDriveMotorReversed[FRONT_RIGHT],
					kSpinningMotorReversed[FRONT_RIGHT],
					kAbsEncoderChannel[FRONT_RIGHT],
					kOffsetAngle[FRONT_RIGHT]),
			new SwerveModule(
					kDriveMotorId[BACK_LEFT],
					kSpinningMotorId[BACK_LEFT],
					kDriveMotorReversed[BACK_LEFT],
					kSpinningMotorReversed[BACK_LEFT],
					kAbsEncoderChannel[BACK_LEFT],
					kOffsetAngle[BACK_LEFT]),
			new SwerveModule(
					kDriveMotorId[BACK_RIGHT],
					kSpinningMotorId[BACK_RIGHT],
					kDriveMotorReversed[BACK_RIGHT],
					kSpinningMotorReversed[BACK_RIGHT],
					kAbsEncoderChannel[BACK_RIGHT],
					kOffsetAngle[BACK_RIGHT])};

	private final AHRS _gyro = new AHRS(SPI.Port.kMXP);
	private final PIDController xAutoController = new PIDController(kPXAuto, 0, 0);
	private final PIDController yAutoController = new PIDController(kPYAuto, 0, 0);
	private final ProfiledPIDController thetaController = new ProfiledPIDController(
			kPThetaAuto,
			0,
			0,
			kThetaControllerConstraints);

  private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
        kSwerveKinematics,
        getRotation2d(),
        new SwerveModulePosition[]{
              swerveModules[FRONT_LEFT].getPosition(),
              swerveModules[FRONT_RIGHT].getPosition(),
              swerveModules[BACK_LEFT].getPosition(),
              swerveModules[BACK_RIGHT].getPosition()},
        new Pose2d(new Translation2d(3, 4), new Rotation2d(0)));

  private final Limelight ll = new Limelight();

  private final Field2d field = new Field2d();

  public Swerve() {
    resetGyro();
    var tab = Shuffleboard.getTab("Swerve");

    tab.add("FL", swerveModules[FRONT_LEFT]).withWidget(BuiltInWidgets.kGyro);
    tab.add("FR", swerveModules[FRONT_RIGHT]).withWidget(BuiltInWidgets.kGyro);
    tab.add("BL", swerveModules[BACK_LEFT]).withWidget(BuiltInWidgets.kGyro);
    tab.add("BR", swerveModules[BACK_RIGHT]).withWidget(BuiltInWidgets.kGyro);
  }

	public void resetGyro() {
		_gyro.reset();
	}

	private double getDegrees() {
		return Math.IEEEremainder(_gyro.getAngle(), 360);
	}

	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(getDegrees());
	}

  private double getRampAngle(){
    return _gyro.getPitch();
  }

  public Command resetGyroCommand() {
    return new InstantCommand(this::resetGyro);
  }

	public Command resetModulesCommand() {
		return new FunctionalCommand(
				() -> {},
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
			new PrintCommand("began dirve command"),
		Commands.runEnd(
				() -> {
					//create the speeds for x,y and spinning and using a deadBand and Limiter to fix edge cases
					double xSpeed = xLimiter.calculate(applyDeadband(xSpeedSupplier.getAsDouble(), kDeadband)) * kMaxDriveSpeed,
							ySpeed = yLimiter.calculate(applyDeadband(ySpeedSupplier.getAsDouble(), kDeadband)) * kMaxDriveSpeed,
							spinningSpeed = spinningLimiter.calculate(applyDeadband(spinningSpeedSupplier.getAsDouble(), kDeadband)) * kMaxDriveTurningSpeed;

					// create a CassisSpeeds object and apply it the speeds
					ChassisSpeeds chassisSpeeds = fieldOriented.getAsBoolean() ?
							ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, spinningSpeed, getRotation2d()) :
							new ChassisSpeeds(xSpeed, ySpeed, spinningSpeed);

					//use the ChassisSpeedsObject to create an array of SwerveModuleStates
					SwerveModuleState moduleStates[] = kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

					//apply the array to the swerve modules of the robot
					setModulesStates(moduleStates);
				},
				() -> {
					stopModules();
					resetEncoders();
				},
				this));
	}

	public Command driveSwerveWithAngleCommand(
				DoubleSupplier xSpeed,
				DoubleSupplier ySpeed,
				DoubleSupplier xAngle,
				DoubleSupplier yAngle,
				BooleanSupplier fieldOriented) {
		return driveSwerveCommand(
					xSpeed,
					ySpeed,
					() -> thetaController.calculate(
								getDegrees(), calculateJoystickAngle(
											xAngle.getAsDouble(),
											yAngle.getAsDouble())),
					fieldOriented);
	}

	private double calculateJoystickAngle(double x, double y) {
		double a =  -Math.toDegrees(Math.atan(y / x));
		if (x<0) a+=180;
		if (a<0) a+=360;
		return a;
	}

  @Override
  public void periodic() {
    ll.updateFromAprilTagPose(odometry::addVisionMeasurement);
    odometry.update(
          getRotation2d(),
          new SwerveModulePosition[]{
                swerveModules[FRONT_LEFT].getPosition(),
                swerveModules[FRONT_RIGHT].getPosition(),
                swerveModules[BACK_LEFT].getPosition(),
                swerveModules[BACK_RIGHT].getPosition()
          });

		field.setRobotPose(odometry.getEstimatedPosition());
    SmartDashboard.putData(field);

	SmartDashboard.putData("gyro angle", _gyro);
  }


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

  private void stopModules() {
    swerveModules[FRONT_LEFT].stop();
    swerveModules[FRONT_RIGHT].stop();
    swerveModules[BACK_LEFT].stop();
    swerveModules[BACK_RIGHT].stop();
  }

  // autonomous code

  public Command followTrajectoryCommand(Pose2d start, List<Translation2d> wayPoints, Pose2d end) {
    return followTrajectoryCommand(
          TrajectoryGenerator.generateTrajectory(
                start,
                wayPoints,
                end,
                kConfig)
    );
  }

  public Command turnToAngleCommand(double degrees) {
    return driveSwerveCommand(() -> 0,
          () -> 0,
          () -> thetaController.calculate(getDegrees(), degrees),
          () -> true);
  }

  public Command followTrajectoryCommand(Trajectory trajectory) {
    return new SwerveControllerCommand(
          trajectory,
          odometry::getEstimatedPosition,
          kSwerveKinematics,
          xAutoController,
          yAutoController,
          thetaController,
          this::setModulesStates,
          this);
  }

  public Command spinVectorTo(Pose2d desiredPose) {
    return followTrajectoryCommand(odometry.getEstimatedPosition(), new ArrayList<>(), desiredPose);
  }

  public Command vectorTo(Translation2d desiredTranslation) {
    return spinVectorTo(new Pose2d(desiredTranslation, getRotation2d()));
  }

    //private double time = 0;
  //@Override
//  public void periodic() {
//    if (Timer.getFPGATimestamp() - time > 10 && swerveModules[SwerveModule.FRONT_LEFT].getDriveVelocity() < 0.01){
//      swerveModules[SwerveModule.FRONT_LEFT].resetSpinningEncoder();
//      swerveModules[FRONT_RIGHT].resetSpinningEncoder();
//      swerveModules[BACK_LEFT].resetSpinningEncoder();
//      swerveModules[BACK_RIGHT].resetSpinningEncoder();
//
//      time = Timer.getFPGATimestamp();
//    }
//	}
}
