package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import static frc.robot.Constants.SwerveConstants.kTolerance;
import static java.lang.Math.PI;

public class SwerveModule implements Sendable {
  //create the module's motors
  private final CANSparkMax _driveMotor;
  private final CANSparkMax _spinningMotor;

  //create the module's encoders
  private final RelativeEncoder _driveEncoder;
  private final RelativeEncoder _spinningEncoder;
  private final DutyCycleEncoder _absEncoder;
  private final double _resetOffset;
  private final double _absEncoderOffsetRad;
  //a pid controller for the angle of the module
  private final PIDController _spinningPIDController;

  public Trigger isReset = new Trigger(()-> Math.abs(getResetRad()) < kTolerance).debounce(0.1);

  // construct the class
  public SwerveModule(
          int driveMotorId,
          int spinningMotorId,
          boolean driveMotorReversed,
          boolean spinningMotorReversed,
          int absEncoderChannel,
          double offsetAngle) {
    _absEncoder = new DutyCycleEncoder(absEncoderChannel);
    _absEncoderOffsetRad = offsetAngle * 2 * PI;
    _resetOffset = _absEncoderOffsetRad - PI;

    _driveMotor = new CANSparkMax(driveMotorId, CANSparkMax.MotorType.kBrushless);
    _spinningMotor = new CANSparkMax(spinningMotorId, CANSparkMax.MotorType.kBrushless);

    _driveMotor.setInverted(driveMotorReversed);
    _spinningMotor.setInverted(spinningMotorReversed);

    _driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    _spinningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    _driveMotor.clearFaults();
    _spinningMotor.clearFaults();

    _spinningMotor.setSmartCurrentLimit(20);
    _driveMotor.setSmartCurrentLimit(60);

    _driveEncoder = _driveMotor.getEncoder();
    _spinningEncoder = _spinningMotor.getEncoder();

    _driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRotationsToMeters);
    _driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPMToMeterPerSec);
    _spinningEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderRotationsToRadians);
    _spinningEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderRPMToRadiansPerSec);

    _spinningPIDController = new PIDController(Constants.ModuleConstants.kPTurning, 0, 0);
    _spinningPIDController.enableContinuousInput(-PI, PI);

    resetEncoders();
  }

  public double getDrivePosition() {
    return _driveEncoder.getPosition();
  }

  public double getSpinningPosition() {
    return _spinningEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return _driveEncoder.getVelocity();
  }

  public double getSpinningVelocity() {
    return _spinningEncoder.getVelocity();
  }

  // return the module angle between -PI to PI
  public double getResetRad() {
    double angle = _absEncoder.getAbsolutePosition();
    angle = angle * 2 * PI -PI;
    angle -= _resetOffset;
    angle = angle < -PI ? 2 * PI + angle : angle;
    angle = angle > PI ? angle - (2 * PI) : angle;
    return angle;
  }

  // return the module angle between 0 to 2PI
  public double getAbsEncoderRad() {
    double angle = _absEncoder.getAbsolutePosition();
    angle = angle * 2 * PI;
    angle -= _absEncoderOffsetRad;
    angle = angle < 0 ? 2 * PI + angle : angle;
    return angle;
  }

  public void resetEncoders() {
//    _driveEncoder.setPosition(0);
    _spinningEncoder.setPosition(getAbsEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSpinningPosition()));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getSpinningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      stopModule();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);

    _driveMotor.set(state.speedMetersPerSecond / Constants.SwerveConstants.kPhysicalMaxSpeedMetersPerSecond);
    _spinningMotor.set(_spinningPIDController.calculate(getSpinningPosition(), state.angle.getRadians()));
  }

  public void spinTo(double setpoint){
    if (Math.abs(getResetRad() - setpoint) > kTolerance) {
      _spinningMotor.set(_spinningPIDController.calculate(setpoint, getResetRad()));
    }
    else {
      _spinningMotor.set(0);
    }
  }

  public double getAbsPos(){
    return _absEncoder.getAbsolutePosition();
  }

  public void stopModule() {
    _driveMotor.set(0);
    _spinningMotor.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", () -> Math.toDegrees(getAbsEncoderRad()), null);
    builder.addDoubleProperty("absEncoderPos", this::getAbsPos, null);
    builder.addDoubleProperty("drive output current", _driveMotor::getOutputCurrent, null);
    builder.addDoubleProperty("drive dc output", _driveMotor::getAppliedOutput, null);
  }
}
