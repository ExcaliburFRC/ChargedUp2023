package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
  private final CANSparkMax angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax angleFollowerMotor = new CANSparkMax(ANGLE_FOLLOWER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax lengthMotor = new CANSparkMax(LENGTH_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final RelativeEncoder lengthEncoder = lengthMotor.getEncoder();

  private final DutyCycleEncoder absAngleEncoder = new DutyCycleEncoder(ABS_ANGLE_ENCODER_CHANNEL);

  private final DigitalInput lowerLimitSwitch = new DigitalInput(CLOSED_LIMIT_SWITCH_ID);
//  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);

  private final Trigger armFullyClosedTrigger = new Trigger(() -> !lowerLimitSwitch.get());
//  private final Trigger armHalfOpenedTrigger = new Trigger(() -> !lowerLimitSwitch.get());
  private final Trigger armFullyOpenedTrigger = new Trigger(() -> lengthEncoder.getPosition() > 1);

  private final Trigger armClosedTrigger= new Trigger(()-> angleInRange(CLOSED_DEGREES, absAngleEncoder.getDistance()));

  private final PIDController angleController = new PIDController(
        kP_ANGLE, 0, 0);

  private final SparkMaxPIDController lengthController = lengthMotor.getPIDController();

  public static double floatDutyCycle = 0;

  public Arm() {
    angleFollowerMotor.restoreFactoryDefaults();
    angleMotor.restoreFactoryDefaults();
    lengthMotor.restoreFactoryDefaults();

    angleMotor.setInverted(false);
    angleFollowerMotor.follow(angleMotor, false);
    lengthMotor.setInverted(true);


    angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    angleFollowerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lengthMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    lengthEncoder.setPositionConversionFactor(ROT_TO_METER);
    lengthEncoder.setVelocityConversionFactor(RPM_TO_METER_PER_SEC);

    lengthController.setP(kP_LENGTH);
    lengthController.setI(0);
    lengthController.setD(kD_LENGTH);


    absAngleEncoder.setPositionOffset(ABS_ENCODER_OFFSET_ANGLE_DEG);
    absAngleEncoder.setDistancePerRotation(360.0);
  }

  /**
   * manual control of the system using the controller's joysticks
   *
   * @param angleJoystick  a supplier of the speed to input to the arm's angle motor's
   * @param lengthJoystick a supplier of the speed to input to the arm's length motor
   * @return the command
   */
  public Command joystickManualCommand(DoubleSupplier angleJoystick, DoubleSupplier lengthJoystick) {
    return new RunCommand(
          () -> {
            lengthMotor.set(-lengthJoystick.getAsDouble() / 2);
            angleMotor.set(-angleJoystick.getAsDouble() / 4);

            floatDutyCycle = angleJoystick.getAsDouble() / 4;
          }, this);
  }

  /**
   * manual control of the system using a joystick and the POV buttons
   *
   * @param angleJoystick a supplier of the speed to input to the arm's angle motor's
   * @param lengthPOV     a supplier of the POV angle from the joystick
   * @return the command
   */
  public Command povManualCommand(DoubleSupplier angleJoystick, DoubleSupplier lengthPOV) { //kimmel
    return new RunCommand(() -> {
      if (lengthPOV.getAsDouble() == -1) lengthMotor.set(0);
      else {
        if (lengthPOV.getAsDouble() == 0) // pov up is pressed
          lengthMotor.set(0.4);
        if (lengthPOV.getAsDouble() == 180) // pov down is pressed
          lengthMotor.set(-0.6);
      }

      angleMotor.set(angleJoystick.getAsDouble() / 4);
      floatDutyCycle = MathUtil.clamp(angleJoystick.getAsDouble(), -1, 0);
    }, this);
  }

  /**
   * moves the arm's length in a given speed
   *
   * @param lengthSpeed the speed to move the length motor in
   * @return the command
   */
  public Command manualLengthCommand(DoubleSupplier lengthSpeed) {
    return new RunCommand(() -> lengthMotor.set(lengthSpeed.getAsDouble() / 2));
  }

  public Command resetLengthCommand() {
    return this.runEnd(
          () -> lengthMotor.set(-0.75),
          lengthMotor::stopMotor
    ).until(armFullyClosedTrigger);
  }

  public Command holdSetpointCommand(Translation2d setpoint) {
    return resetLengthCommand().andThen(
          moveToLengthCommand(setpoint).alongWith(moveToAngleCommand(setpoint)))
          .withName("hold setpoint command");
  }

  public Command moveToLengthCommand(Translation2d setPoint) {
    return new ProxyCommand(
          () -> new TrapezoidProfileCommand(
                new TrapezoidProfile(
                      new TrapezoidProfile.Constraints(kMaxLinearVelocity, kMaxLinearAcceleration),
                      new TrapezoidProfile.State(setPoint.getNorm(), 0),
                      new TrapezoidProfile.State(lengthEncoder.getPosition(), lengthEncoder.getVelocity())),
                state -> {
                  double feedforward = kS_LENGTH * Math.signum(state.velocity)
                        + kG_LENGTH * Math.sin(Units.degreesToRadians(absAngleEncoder.getDistance()))
                        + kV_LENGTH * state.velocity;

                  lengthController.setReference(state.position, CANSparkMax.ControlType.kPosition,
                        0,
                        feedforward, SparkMaxPIDController.ArbFFUnits.kVoltage);
                }))
          .finallyDo((__)-> lengthMotor.stopMotor());
  }

  public Command moveToAngleCommand(Translation2d setpoint) {
    return this.run(()-> {
      double pid = angleController.calculate(absAngleEncoder.getDistance(), setpoint.getAngle().getDegrees());
      double feedforward = kS_ANGLE * Math.signum(pid) + kG_ANGLE * setpoint.getAngle().getCos();

      angleMotor.setVoltage(pid + feedforward);

      SmartDashboard.putNumber("pid", pid);
      SmartDashboard.putNumber("feedforward", feedforward);
      SmartDashboard.putNumber("sum", pid + feedforward);
    })
          .finallyDo((__)-> angleMotor.stopMotor());
  }

  /**
   * floats the arm using the current DutyCycle
   *
   * @return the command
   */
  public Command floatCommand() {
    return new RunCommand(
          () -> {
            angleMotor.set(floatDutyCycle);
          }, this
    );
  }

  private boolean angleInRange(double angleA, double angleB){
    double tolorance = 5;
    return Math.abs(angleA - angleB) < tolorance;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");
    builder.addBooleanProperty("fully closed", armFullyClosedTrigger, null);
    builder.addDoubleProperty("arm angle", absAngleEncoder::getDistance, null);
    builder.addDoubleProperty("arm length", lengthEncoder::getPosition, null);
  }

  @Override
  public void periodic() {
    if (armFullyClosedTrigger.getAsBoolean()) lengthEncoder.setPosition(MINIMAL_LENGTH_METERS);

    if (DriverStation.isTest()) {
      angleMotor.setVoltage(SmartDashboard.getNumber("volts", 0));
    }
  }
}
