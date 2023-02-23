package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utiliy.Calculation;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
  private final CANSparkMax angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax angleFollowerMotor = new CANSparkMax(ANGLE_FOLLOWER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax lengthMotor = new CANSparkMax(LENGTH_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final RelativeEncoder lengthEncoder;

  private final DutyCycleEncoder absAngleEncoder = new DutyCycleEncoder(ABS_ANGLE_ENCODER_CHANNEL);

  private final DigitalInput lowerLimitSwitch = new DigitalInput(CLOSED_LIMIT_SWITCH_ID);

  private final Trigger armFullyClosedTrigger = new Trigger(() -> !lowerLimitSwitch.get());
  private final Trigger armFullyOpenedTrigger = new Trigger(() -> !lowerLimitSwitch.get());

  public Arm() {
    angleFollowerMotor.restoreFactoryDefaults();
    angleMotor.restoreFactoryDefaults();
    lengthMotor.restoreFactoryDefaults();

    angleFollowerMotor.follow(angleMotor, false);
    angleMotor.setInverted(true);
    lengthMotor.setInverted(true);

    angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lengthMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    angleFollowerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    lengthEncoder = lengthMotor.getEncoder();

    lengthEncoder.setPositionConversionFactor(ROT_TO_METER);
    lengthEncoder.setVelocityConversionFactor(RPM_TO_METER_PER_SEC);
  }

  public Command joystickManualCommand(DoubleSupplier angleJoystick, DoubleSupplier lengthJoystick) {
    return new RunCommand(
          () -> {
            lengthMotor.set(lengthJoystick.getAsDouble());
            angleMotor.set(angleJoystick.getAsDouble());

            Calculation.floatDutyCycle = MathUtil.clamp(angleJoystick.getAsDouble(), -1, 0);
          }, this);
  }

  public Command povManualCommand(DoubleSupplier angleJoystick, DoubleSupplier lengthPOV) { //kimmel
    return new RunCommand(() -> {
      if (lengthPOV.getAsDouble() == -1) lengthMotor.set(0);
      else {
        if (lengthPOV.getAsDouble() == 0) // pov up is pressed
          lengthMotor.set(0.25);
        if (lengthPOV.getAsDouble() == 180) // pov down is pressed
          lengthMotor.set(-0.45);
      }

      angleMotor.set(angleJoystick.getAsDouble());
      Calculation.floatDutyCycle = MathUtil.clamp(angleJoystick.getAsDouble(), -1, 0);
    }, this);
  }

  public Command closeArmCommand(){
    return new RunCommand(()-> lengthMotor.set(-0.2) ,this).until(armFullyClosedTrigger);
  }

  public Command holdArmCommand(double dc, BooleanSupplier accel, double telescope, double angle) {
    return closeArmCommand().andThen(
          new RunCommand(() -> {
      double a = accel.getAsBoolean() ? 0.2 : 0;
      angleMotor.set(dc + a);
    }, this).until(()-> angleInRange(angle)),
                new RunCommand(() -> {
                  angleMotor.set(dc);
                    if (telescope < lengthEncoder.getPosition())
                      lengthMotor.set(-0.2);
                    else lengthMotor.set(0.2);
                }).until(() -> lengthInRange(telescope))
          );
  }

  private boolean lengthInRange(double setpoint) {
    double tolorance = 0.3; //TODO: find
    return Math.abs(setpoint - lengthEncoder.getPosition()) < tolorance;
  }

  private boolean angleInRange(double setpoint) {
    double tolorance = 0.3; //TODO: find
    return Math.abs(setpoint - getArmDegrees()) < tolorance;
  }

  private double getArmDegrees() {
    double wantedAngle = absAngleEncoder.getAbsolutePosition() - ABS_ENCODER_OFFSET_ANGLE_DEG;
    if (wantedAngle < 0) wantedAngle += 1;
    wantedAngle *= -1;
    wantedAngle += 1;
    return wantedAngle * 360;
  }

  public Command floatCommand() {
    return new RunCommand(
          () -> {
            angleMotor.set(Calculation.floatDutyCycle);
            System.out.println(Calculation.floatDutyCycle);
          },
          this
    );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");
    builder.addBooleanProperty("fully closed", armFullyClosedTrigger, null);
    builder.addDoubleProperty("arm angle", this::getArmDegrees, null);
  }
}
