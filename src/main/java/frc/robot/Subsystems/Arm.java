package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
  private final CANSparkMax angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax lengthMotor = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final RelativeEncoder lengthEncoder;

  private final DutyCycleEncoder absAngleEncoder = new DutyCycleEncoder(ABS_ANGLE_ENCODER_CHANNEL);

  private final DigitalInput upperLimitSwitch = new DigitalInput(UPPER_LIMIT_SWITCH_ID);
  private final DigitalInput lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_SWITCH_ID);

  private final Trigger armFullyOpenedTrigger = new Trigger(() -> !upperLimitSwitch.get());
  private final Trigger armFullyClosedTrigger = new Trigger(() -> !lowerLimitSwitch.get());

  private final PIDController angleController;
  private final SparkMaxPIDController lengthController;
//  private final PIDController lengthController;


  public Arm() {
    angleMotor.restoreFactoryDefaults();
    lengthMotor.restoreFactoryDefaults();

    angleMotor.setInverted(false); //TODO: check
    lengthMotor.setInverted(false); //TODO: check

    angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lengthMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    lengthEncoder = lengthMotor.getEncoder();

    lengthEncoder.setPositionConversionFactor(ROT_TO_METER);
    lengthEncoder.setVelocityConversionFactor(RPM_TO_METER_PER_SEC);

    lengthController = new PIDController(kP_LENGTH, kI_LENGTH, kD_LENGTH);
    angleController = new PIDController(kP_ANGLE, kI_ANGLE, kD_ANGLE);


    angleController.setTolerance(ANGLE_TOLERANCE);

    angleMotor.setOpenLoopRampRate(ARM_RAMP_RATE);
  }

  public Command manualCommand(DoubleSupplier angleJoystick, DoubleSupplier lengthJoystick) {
    return new RunCommand(
          () -> {
            lengthMotor.set(lengthJoystick.getAsDouble());
            angleMotor.set(angleJoystick.getAsDouble());
          }, this);
  }

  public double getLengthMeter() {
    return MINIMAL_LENGTH_METERS + lengthEncoder.getPosition();
  }

  public Command resetLengthEncoder() {
    return new RunCommand(
          () -> lengthMotor.set(-0.05)
    ).until(
                armFullyClosedTrigger)
          .andThen(new InstantCommand(
                () -> {
                  lengthEncoder.setPosition(0);
                  lengthMotor.stopMotor();
                }
          ));
  }

  public Command setTranslationCommand(Setpoints setPointConstant) {
    Translation2d setPoint = setPointConstant.getTranslation2d();
    Command lengthCommand = openToLengthCommand(setPoint);

    double lengthSetPoint = setPoint.getNorm();
    double angleSetPoint = setPoint.getNorm();
    return new RunCommand(
          () -> {
            angleMotor.set(angleController.calculate(
                  getAbsEncoderPos(),
                  angleSetPoint));
          }).alongWith(new ProxyCommand(()-> openToLengthCommand(setPoint))
          .unless(() -> isAchievableTranslation(setPoint));
  }

  private Command openToLengthCommand(Translation2d setPoint) {
    return new ProxyCommand( () -> new TrapezoidProfileCommand(
          new TrapezoidProfile(
                new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration),
                new TrapezoidProfile.State(setPoint.getNorm(), 0),
                new TrapezoidProfile.State(getLengthMeter(), lengthEncoder.getVelocity())
          ),
          state -> {
            double feedforward = ks * Math.signum(state.velocity)
                  + kg * Math.sin(Units.degreesToRadians(getAbsEncoderPos()))
                  + kv * state.velocity;

            lengthController.setReference(state.position, CANSparkMax.ControlType.kPosition,
                  0,
                  feedforward, SparkMaxPIDController.ArbFFUnits.kVoltage);
          }, this
    ));
  }

  private boolean isAchievableTranslation(Translation2d target) {
    return target.getNorm() >= MINIMAL_LENGTH_METERS && target.getNorm() <= MINIMAL_LENGTH_METERS * 2 &&
          (target.getAngle().getDegrees() <= PHYSICAL_BACK_MAX_ARM_ANGLE_DEG ||
                target.getAngle().getDegrees() >= PHYSICAL_FRONT_MAX_ARM_ANGLE_DEG);
  } // TODO: find the max length multiplier

    private double getAbsEncoderPos() {
        return absAngleEncoder.getAbsolutePosition() - ABS_ENCODER_OFFSET_ANGLE_DEG < 0 ?
                360 - (absAngleEncoder.getAbsolutePosition() + ABS_ENCODER_OFFSET_ANGLE_DEG) :
                absAngleEncoder.getAbsolutePosition() - ABS_ENCODER_OFFSET_ANGLE_DEG;
    }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("fullyOpened", armFullyOpenedTrigger, null);
    builder.addBooleanProperty("fullyClosed", armFullyClosedTrigger, null);
  }
}
