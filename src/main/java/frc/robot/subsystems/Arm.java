package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.utility.Calculation;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
  private final CANSparkMax angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax angleFollowerMotor = new CANSparkMax(ANGLE_FOLLOWER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax lengthMotor = new CANSparkMax(LENGTH_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final RelativeEncoder lengthEncoder = lengthMotor.getEncoder();

  private final DutyCycleEncoder absAngleEncoder = new DutyCycleEncoder(ABS_ANGLE_ENCODER_CHANNEL);

  private final DigitalInput lowerLimitSwitch = new DigitalInput(CLOSED_LIMIT_SWITCH_ID);

  private final Trigger armFullyClosedTrigger = new Trigger(() -> !lowerLimitSwitch.get());

  private final SparkMaxPIDController lengthController = lengthMotor.getPIDController();
  private final SparkMaxPIDController angleController = angleMotor.getPIDController();

  public Arm() {
    angleFollowerMotor.restoreFactoryDefaults();
    angleMotor.restoreFactoryDefaults();
    lengthMotor.restoreFactoryDefaults();

    angleFollowerMotor.follow(angleMotor, false); // TODO: check
    angleMotor.setInverted(false); //TODO: check
    lengthMotor.setInverted(false); //TODO: check

    angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lengthMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    angleFollowerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    lengthEncoder.setPositionConversionFactor(ROT_TO_METER);
    lengthEncoder.setVelocityConversionFactor(RPM_TO_METER_PER_SEC);

    lengthController.setP(Constants.ArmConstants.kP_LENGTH);
    lengthController.setI(0);
    lengthController.setD(0);

    angleController.setP(kP_ANGLE);
    angleController.setI(0);
    angleController.setD(0);

    // kG_ANGLE.put(MINIMAL_LENGTH_METERS, SysId:kG);
    // kG_ANGLE.put(MAXIMAL_LENGTH_METERS / 2, SysId:kG);
    // kG_ANGLE.put(MAXIMAL_LENGTH_METERS, SysId:kG);

    angleMotor.setClosedLoopRampRate(ARM_RAMP_RATE);
  }


  /**
   * manual control of the system using the controller's joysticks
   * @param angleJoystick a supplier of the speed to input to the arm's angle motor's
   * @param lengthJoystick a supplier of the speed to input to the arm's length motor
   * @return the command
   */
  public Command joystickManualCommand(DoubleSupplier angleJoystick, DoubleSupplier lengthJoystick) {
    return new RunCommand(
          () -> {
            lengthMotor.set(lengthJoystick.getAsDouble() / 4);
            angleMotor.set(angleJoystick.getAsDouble() / 4);

            Calculation.floatDutyCycle = MathUtil.clamp(angleJoystick.getAsDouble(), -1, 0);
          }, this);
  }

  /**
   * manual control of the system using a joystick and the POV buttons
   * @param angleJoystick a supplier of the speed to input to the arm's angle motor's
   * @param lengthPOV a supplier of the POV angle from the joystick
   * @return the command
   */
  public Command povManualCommand(DoubleSupplier angleJoystick, DoubleSupplier lengthPOV) { //kimmel
    return new RunCommand(() -> {
      if (lengthPOV.getAsDouble() == -1) lengthMotor.set(0);
      else {
        if (lengthPOV.getAsDouble() == 0) // pov up is pressed
          lengthMotor.set(0.15);
        if (lengthPOV.getAsDouble() == 180) // pov down is pressed
          lengthMotor.set(-0.35);
      }

      angleMotor.set(angleJoystick.getAsDouble() / 4);
      Calculation.floatDutyCycle = MathUtil.clamp(angleJoystick.getAsDouble(), -1, 0);
    }, this);
  }

  /**
   * moves the arm's length in a given speed
   * @param lengthSpeed the speed to move the length motor in
   * @return the command
   */
  public Command manualLengthCommand(DoubleSupplier lengthSpeed){
    return new RunCommand(()-> lengthMotor.set(lengthSpeed.getAsDouble()));
  }

  public Command calibrateLengthEncoderCommand() {
    return this.runEnd(
          ()-> lengthMotor.set(-0.3),
          lengthMotor::stopMotor);
  }

  public Command holdSetpointCommand(Translation2d setpoint) {
    return moveToLengthCommand(setpoint).alongWith(moveToAngleCommand(setpoint));
  }

  private Command moveToLengthCommand(Translation2d setPoint) {
    return new ProxyCommand(
          () -> new TrapezoidProfileCommand(
                new TrapezoidProfile(
                      new TrapezoidProfile.Constraints(kMaxLinearVelocity, kMaxAngularAcceleration),
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

  /**
   * floats the arm using the current DutyCycle
   * @return the command
   */
  public Command floatCommand() {
    return new RunCommand(
          () -> {
            angleMotor.set(Calculation.floatDutyCycle);
            System.out.println(Calculation.floatDutyCycle);
          },
          this
    );
  }

  private Command disableArmCommand(){
    return new RunCommand(angleMotor::stopMotor, this).until(()-> getArmDegrees() > 180)
          .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");
    builder.addBooleanProperty("fully closed", armFullyClosedTrigger, null);
    builder.addDoubleProperty("arm angle", this::getArmDegrees, null);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm angle", getArmDegrees());
    SmartDashboard.putBoolean("fully closed", armFullyClosedTrigger.getAsBoolean());
    SmartDashboard.putNumber("length", lengthEncoder.getPosition());
    if (armFullyClosedTrigger.getAsBoolean()) lengthEncoder.setPosition(0);

//    if (getArmDegrees() < 180) disableArmCommand().schedule();
  }
}
