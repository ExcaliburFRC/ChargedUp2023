package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utiliy.Calculation;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
  private final CANSparkMax angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax angleFollowerMotor = new CANSparkMax(ANGLE_FOLLOWER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax lengthMotor = new CANSparkMax(LENGTH_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final RelativeEncoder lengthEncoder;
  private final RelativeEncoder tempAngleEncoder = angleMotor.getEncoder();

  private final DutyCycleEncoder absAngleEncoder = new DutyCycleEncoder(ABS_ANGLE_ENCODER_CHANNEL);

  private final DigitalInput upperLimitSwitch = new DigitalInput(UPPER_LIMIT_SWITCH_ID);
  private final DigitalInput lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_SWITCH_ID);

  private final Trigger armFullyOpenedTrigger = new Trigger(() -> !upperLimitSwitch.get());
  private final Trigger armFullyClosedTrigger = new Trigger(() -> !lowerLimitSwitch.get());

  AtomicInteger floatDutyCycle = new AtomicInteger(0);

  private final SparkMaxPIDController angleController;
  private final SparkMaxPIDController lengthController;

  public Arm() {
    angleFollowerMotor.restoreFactoryDefaults();
    angleMotor.restoreFactoryDefaults();
    lengthMotor.restoreFactoryDefaults();

    angleFollowerMotor.follow(angleMotor, false);
    angleMotor.setInverted(true);
    lengthMotor.setInverted(true); //TODO: check

    angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lengthMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    angleFollowerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    lengthEncoder = lengthMotor.getEncoder();
    tempAngleEncoder.setPositionConversionFactor(tempCoversion);

    lengthEncoder.setPositionConversionFactor(ROT_TO_METER);
    lengthEncoder.setVelocityConversionFactor(RPM_TO_METER_PER_SEC);

    lengthController = lengthMotor.getPIDController();

    lengthController.setP(kP_LENGTH);
    lengthController.setI(0);
    lengthController.setD(0);

    angleController = angleMotor.getPIDController();
    angleController.setP(kP_ANGLE);
    angleController.setI(0);
    angleController.setD(0);
    angleController.setD(0);

    angleMotor.setClosedLoopRampRate(ARM_RAMP_RATE);
  }

  public Command manualCommand(DoubleSupplier angleJoystick, DoubleSupplier lengthJoystick) {
    return new RunCommand(
          () -> {
            lengthMotor.set(lengthJoystick.getAsDouble());
            angleMotor.set(angleJoystick.getAsDouble());
            Calculation.floatDutyCycle = angleJoystick.getAsDouble();
            System.out.println(Calculation.floatDutyCycle);
          }, this);
  }

  public Command calibrateLengthEncoderCommand() {
    return new RunCommand(
          () -> lengthMotor.set(-0.1))
          .until(armFullyClosedTrigger)
          .andThen(new InstantCommand(
                () -> {
                  lengthMotor.stopMotor();
                  lengthEncoder.setPosition(MINIMAL_LENGTH_METERS);
                }
          ));
  }

  private double getArmDegrees() {
    double wantedAngle = absAngleEncoder.getAbsolutePosition() - ABS_ENCODER_OFFSET_ANGLE_DEG;
    if (wantedAngle < 0) wantedAngle += 1;
    wantedAngle *= -1;
    wantedAngle += 1;
    return wantedAngle * 360;
  }

  public Command holdSetpointCommand(Translation2d setpoint) {
    return new SequentialCommandGroup(
          moveToLengthCommand(new Translation2d(MINIMAL_LENGTH_METERS, new Rotation2d(0))),
          moveToAngleCommand(setpoint),
          moveToLengthCommand(setpoint)
    );
  }

  private Command moveToLengthCommand(Translation2d setPoint) {
    return new ProxyCommand(
          () -> new TrapezoidProfileCommand(
                new TrapezoidProfile(
                      new TrapezoidProfile.Constraints(kMaxLinearVelocity, kMaxLinearAcceleration),
                      new TrapezoidProfile.State(setPoint.getNorm(), 0),
                      new TrapezoidProfile.State(lengthEncoder.getPosition(), lengthEncoder.getVelocity())),
                state -> {
                  double feedforward = kS_LENGTH * Math.signum(state.velocity)
                        + kG_LENGTH * Math.sin(Units.degreesToRadians(getArmDegrees()))
                        + kV_LENGTH * state.velocity;

                  lengthController.setReference(state.position, CANSparkMax.ControlType.kPosition,
                        0,
                        feedforward, SparkMaxPIDController.ArbFFUnits.kVoltage);
                }, this));
  }

  public Command moveToAngleCommand(Translation2d setPoint) {
    return new ProxyCommand(
          () -> new TrapezoidProfileCommand(
                new TrapezoidProfile(
                      new TrapezoidProfile.Constraints(kMaxAngularVelocity, kMaxAngularAcceleration),
                      new TrapezoidProfile.State(setPoint.getAngle().getDegrees(), 0),
                      new TrapezoidProfile.State(getArmDegrees(), 0)
                ),
                state -> {
                  double feedforward =
                        kS_ANGLE * Math.signum(state.velocity)
                              + kG_ANGLE.get(lengthEncoder.getPosition()) * Math.cos(state.position)
                              + kV_ANGLE * state.velocity;
                  angleController.setReference(state.position, CANSparkMax.ControlType.kPosition,
                        0,
                        feedforward, SparkMaxPIDController.ArbFFUnits.kVoltage);
                }));
  }

  public Command floatCommand(){
    return new RunCommand(
          ()-> {
            angleMotor.set(Calculation.floatDutyCycle);
            System.out.println(Calculation.floatDutyCycle);
          },
          this
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("magnetic lS", armFullyOpenedTrigger.getAsBoolean());
    SmartDashboard.putBoolean("lS", armFullyClosedTrigger.getAsBoolean());
    SmartDashboard.putNumber("abs angle",absAngleEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("get arm degrees", getArmDegrees());
    SmartDashboard.putNumber("length position", lengthEncoder.getPosition());
    SmartDashboard.putNumber("temp angle position", tempAngleEncoder.getPosition());

    if (armFullyOpenedTrigger.getAsBoolean()) {
      lengthEncoder.setPosition(MAXIMAL_LENGTH_METERS);
      if (lengthMotor.get() > 0) lengthMotor.set(0);
    }

    if (armFullyClosedTrigger.getAsBoolean()) {
      lengthEncoder.setPosition(MINIMAL_LENGTH_METERS);
      if (lengthMotor.get() < 0) lengthMotor.set(0);
    }
  }
}
