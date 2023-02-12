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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.utiliy.Limelight;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
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
    //create gyro
    private final AHRS _gyro = new AHRS(SPI.Port.kMXP);
    //create the pid controllers for auto mode
    private final PIDController xAutoController = new PIDController(kPXAuto, 0, 0);
    private final PIDController yAutoController = new PIDController(kPYAuto, 0, 0);
    private final ProfiledPIDController thetaAutoController = new ProfiledPIDController(
            kPThetaAuto,
            0,
            0,
            kThetaControllerConstraints);
    private final PIDController thetaTeleopController = new PIDController(kPThetaTeleop, 0, kDThetaTeleop);
    //creating th pose estimator
    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
            kSwerveKinematics,
            getRotation2d(),
            new SwerveModulePosition[]{
                    swerveModules[FRONT_LEFT].getPosition(),
                    swerveModules[FRONT_RIGHT].getPosition(),
                    swerveModules[BACK_LEFT].getPosition(),
                    swerveModules[BACK_RIGHT].getPosition()},
            new Pose2d(new Translation2d(3, 4), new Rotation2d(0)));
    //creating the lime light
    private final Limelight ll = new Limelight();
    // the field object
    private final Field2d field = new Field2d();
    //creating an atomicInteger to hold the last angle in the angle based driving technic
    private final AtomicInteger lastJoystickAngle = new AtomicInteger(0);

    //the constructor of the class
    public Swerve() {
        //reset the gyro to 0 degrees in a scale from -180 to 180
        resetGyro();
        //sets some stuff for the pid for the angle based technic
        thetaTeleopController.enableContinuousInput(-180, 180);
        thetaTeleopController.setTolerance(1.5);

        //creating the windows for debug info on the swerve
        var tab = Shuffleboard.getTab("Swerve");
        tab.add("FL", swerveModules[FRONT_LEFT]).withWidget(BuiltInWidgets.kGyro);
        tab.add("FR", swerveModules[FRONT_RIGHT]).withWidget(BuiltInWidgets.kGyro);
        tab.add("BL", swerveModules[BACK_LEFT]).withWidget(BuiltInWidgets.kGyro);
        tab.add("BR", swerveModules[BACK_RIGHT]).withWidget(BuiltInWidgets.kGyro);
    }

    public void resetGyro() {
        _gyro.reset();
    }

    // return the angle of the robot in degrees between 0 and 360
    //TODO: check if works
    private double getDegrees() {
        double deg = Math.IEEEremainder(_gyro.getAngle(), 360);
        deg *= -1;
        if (deg < 0) deg += 360;
        return deg;
    }

    //get the rotation2d of the robots heading
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getDegrees());
    }

    //return the pitch of the robot
    //TODO: check if works
    private double getRampAngle() {
        double pitch = _gyro.getPitch();
        pitch *= -1;
        if (pitch < 0) pitch += 360;
        return pitch;
    }

    // returns a command that resets the gyro to 0
    public Command resetGyroCommand() {
        return new InstantCommand(this::resetGyro);
    }

    //returns a command that spin the modules to a straight pose and resets them
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

    //returns a command that enable the driver to control the swerve at each technic he wants
    public Command dualDriveSwerveCommand(
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier xAngle,
            DoubleSupplier yAngle,
            BooleanSupplier fieldOriented,
            BooleanSupplier withAngle) {
        return new RepeatCommand(
                driveSwerveCommand(
                        xSpeedSupplier, ySpeedSupplier, xAngle, fieldOriented).until(() -> withAngle.getAsBoolean())
                        .andThen(
                                driveSwerveWithAngleCommand(xSpeedSupplier, ySpeedSupplier, () -> -xAngle.getAsDouble(), () -> yAngle.getAsDouble(), fieldOriented).until(() -> !withAngle.getAsBoolean()))
        );
    }

    //control the swerve in a way that the left joystick control the location speeds
    // and the right one controls the angular speed
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
                new PrintCommand("began drive command"),
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

    //control the swerve in a way that the left joystick control the location speeds
    // and the right one's angle is sets as the robots wanted angle
    public Command driveSwerveWithAngleCommand(
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier xAngle,
            DoubleSupplier yAngle,
            BooleanSupplier fieldOriented) {
        return new InstantCommand(() -> lastJoystickAngle.set((int) getDegrees())).andThen(

                new ParallelRaceGroup(
                        new RunCommand(() -> updateJoystickAngle(xAngle.getAsDouble(), yAngle.getAsDouble())),
                        driveSwerveCommand(
                                xSpeed,
                                ySpeed,
                                () -> -thetaTeleopController.calculate(
                                        getDegrees(), lastJoystickAngle.get()),
                                fieldOriented)
                ));
    }

    //calculate the joystick angle, by x and y in a format of 0 to 360
    // while positive x axis  is  the 0 angle and the growing direction is against the cloak
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
        SmartDashboard.putNumber("lastJoystickAngle", lastJoystickAngle.get());
    }
    //sets the swerve modules to the wanted states
    private void setModulesStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, kPhysicalMaxSpeedMetersPerSecond);
        swerveModules[FRONT_LEFT].setDesiredState(states[FRONT_LEFT]);
        swerveModules[FRONT_RIGHT].setDesiredState(states[FRONT_RIGHT]);
        swerveModules[BACK_LEFT].setDesiredState(states[BACK_LEFT]);
        swerveModules[BACK_RIGHT].setDesiredState(states[BACK_RIGHT]);
    }
    //reset the encoders of the swerve modules to 0
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

    // autonomous code:

    //follow the given translations by their order and finishes at the end pose
    public Command followTrajectoryCommand(List<Translation2d> wayPoints, Pose2d end) {
        return followTrajectoryCommand(
                TrajectoryGenerator.generateTrajectory(
                        odometry.getEstimatedPosition(),
                        wayPoints,
                        end,
                        kConfig)
        );
    }

    public Command turnToAngleCommand(double degrees) {
        return driveSwerveCommand(
                () -> 0,
                () -> 0,
                () -> thetaTeleopController.calculate(getDegrees(), degrees),
                () -> false);
    }
    //follows a given trajectory
    public Command followTrajectoryCommand(Trajectory trajectory) {
        return new SwerveControllerCommand(
                trajectory,
                odometry::getEstimatedPosition,
                kSwerveKinematics,
                xAutoController,
                yAutoController,
                thetaAutoController,
                this::setModulesStates,
                this);
    }
    //go in a straight line to the desired point while spinning to the desired angle
    public Command spinVectorTo(Pose2d desiredPose) {
        return followTrajectoryCommand(new ArrayList<>(), desiredPose);
    }
    //go in a straight line to the desired point
    public Command vectorTo(Translation2d desiredTranslation) {
        return spinVectorTo(new Pose2d(desiredTranslation, getRotation2d()));
    }

    public Command spinToCommand(double angle){
        return driveSwerveWithAngleCommand(
                () -> 0, ()-> 0, ()-> Math.cos(angle), ()-> Math.cos(angle), ()-> true);
    }

    public Command rotateToGridCommand(){
        double angle = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)? 180 : 0;
        return spinToCommand(angle);
    }
}
