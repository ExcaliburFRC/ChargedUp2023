package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Map;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.Setpoints.LOCKED;

public class Arm extends SubsystemBase {
  private final CANSparkMax angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax angleFollowerMotor = new CANSparkMax(ANGLE_FOLLOWER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax lengthMotor = new CANSparkMax(LENGTH_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final RelativeEncoder lengthEncoder = lengthMotor.getEncoder();

  private final DutyCycleEncoder absAngleEncoder = new DutyCycleEncoder(ABS_ANGLE_ENCODER_CHANNEL);

  private final DigitalInput lowerLimitSwitch = new DigitalInput(CLOSED_LIMIT_SWITCH_ID);

  public final Trigger armFullyClosedTrigger = new Trigger(() -> !lowerLimitSwitch.get());
  public final Trigger armFullyOpenedTrigger = new Trigger(() -> lengthEncoder.getPosition() >= 1);

  public final Trigger armAngleClosedTrigger = new Trigger(() -> absAngleEncoder.getDistance() <= 92);

  public final Trigger armLockedTrigger = armAngleClosedTrigger.and(armFullyOpenedTrigger);

  private final SparkMaxPIDController lengthController = lengthMotor.getPIDController();

  public static double floatDutyCycle = 0;

  public static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

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

    lengthEncoder.setPosition(MAXIMAL_LENGTH_METERS);

    lengthController.setP(kP_LENGTH);
    lengthController.setI(0);
    lengthController.setD(kD_LENGTH);

    absAngleEncoder.setPositionOffset(ABS_ENCODER_OFFSET_ANGLE_DEG);
    absAngleEncoder.setDistancePerRotation(360.0);

    armTab.addDouble("ArmLength", lengthEncoder::getPosition).withPosition(6, 0)
          .withSize(2, 2).withWidget("Number Slider").withProperties(Map.of("min", MINIMAL_LENGTH_METERS, "max", MAXIMAL_LENGTH_METERS));
    armTab.addDouble("Arm degrees", absAngleEncoder::getDistance).withPosition(4, 0)
          .withWidget("Simple Dial").withProperties(Map.of("min", 90, "max", 190));
    armTab.addBoolean("Fully closed", armFullyClosedTrigger).withPosition(4, 2)
          .withSize(2, 1);
    armTab.addBoolean("Arm locked", armFullyOpenedTrigger.and(armAngleClosedTrigger)).withPosition(4, 3)
          .withSize(2, 1);

//    angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 18f);
//    angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

//    angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -2.5f);
//    angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    angleMotor.setOpenLoopRampRate(2);
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
    return Commands.runEnd(
          () -> lengthMotor.set(-0.85),
          lengthMotor::stopMotor
    ).until(armFullyClosedTrigger);
  }

  public Command holdSetpointCommand(Translation2d setpoint) {
    return moveToLengthCommand(setpoint).alongWith(moveToAngleCommand(setpoint));
  }

  public Command fadeArmCommand() {
    //when the motor stops, gravity slowly pulls the arm down, making the arm "fade" down
    return new RunCommand(angleMotor::stopMotor ,this);
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
          .finallyDo((__) -> lengthMotor.stopMotor());
  }

  public Command moveToAngleCommand(Translation2d setpoint) {
    return Commands.runEnd(() -> {
      double pid = kP_ANGLE * (setpoint.getAngle().getDegrees() - absAngleEncoder.getDistance());
      double feedforward = kS_ANGLE * Math.signum(pid) + kG_ANGLE * setpoint.getAngle().getCos();

         angleMotor.setVoltage(pid + feedforward);
//      SmartDashboard.putNumber("pid", pid);
//      SmartDashboard.putNumber("ff", feedforward);
      SmartDashboard.putNumber("output sum", feedforward + pid);
      SmartDashboard.putNumber("setpoint", setpoint.getAngle().getDegrees());
          }, angleMotor::stopMotor, this);
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

  private boolean angleInRange(double angleA, double angleB) {
    double tolerance = 3;
    return Math.abs(angleA - angleB) < tolerance;
  }

  public Command lockArmCommand(Trigger bbTrigger){
    return moveToAngleCommand(LOCKED.setpoint).until(armAngleClosedTrigger).alongWith(
            resetLengthCommand().andThen(moveToLengthCommand(LOCKED.setpoint).unless(bbTrigger))
    );
  }

  public double getArmLength() {
    return lengthEncoder.getPosition();
  }

  public Command stopAngleMotors(){
    return new InstantCommand(
          ()->{
            angleMotor.disable();
            // disable and stop motor are the same thing
            angleMotor.stopMotor();
            angleFollowerMotor.disable();
            angleFollowerMotor.stopMotor();
          }, this);
  }

  @Override
  public void periodic() {
    if (armFullyClosedTrigger.getAsBoolean()) lengthEncoder.setPosition(MINIMAL_LENGTH_METERS);
  }
}
