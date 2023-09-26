package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  private final DutyCycleEncoder angleEncoder = new DutyCycleEncoder(ABS_ANGLE_ENCODER_PORT);

  private final DigitalInput lowerLimitSwitch = new DigitalInput(CLOSED_LIMIT_SWITCH_ID);

  public final Trigger armFullyClosedTrigger = new Trigger(() -> !lowerLimitSwitch.get());
  public final Trigger armFullyOpenedTrigger = new Trigger(() -> lengthEncoder.getPosition() >= LOCKED_LENGTH_METERS - 0.02);

  public final Trigger armAngleClosedTrigger = new Trigger(() -> getArmAngle() <= 95);

  public final Trigger armLockedTrigger = armAngleClosedTrigger.and(armFullyOpenedTrigger);

  private final SparkMaxPIDController lengthController = lengthMotor.getPIDController();

  public static double floatDutyCycle = 0;

  public static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

  private Translation2d lastHeldSetpoint = new Translation2d();

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

    angleEncoder.setPositionOffset(ABS_ENCODER_OFFSET_ANGLE_DEG);
    angleEncoder.setDistancePerRotation(360);

    lengthController.setP(kP_LENGTH);
    lengthController.setI(0);
    lengthController.setD(kD_LENGTH);

    lengthMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 1.03f);
    lengthMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    angleMotor.setOpenLoopRampRate(1.5);

    lengthEncoder.setPosition(LOCKED_LENGTH_METERS);

    initShuffleboardData();
    setDefaultCommand(fadeArmCommand());
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

  public double getArmAngle() {
    double angle = -angleEncoder.getDistance();

    if (angle > 220 || angle < 80)
      DriverStation.reportError("arm encoder bugged!!", true);
    return MathUtil.clamp(angle, 80, 220);
  }

  public Command blindCloseArmCommand(){
    return resetLengthCommand().andThen(new RunCommand(()-> angleMotor.set(-0.2))
            .finallyDo((__)-> moveToLengthCommand(LOCKED.setpoint).schedule()));
  }

  public Command resetLengthCommand() {
    return Commands.runEnd(
          () -> lengthMotor.set(-0.7),
          lengthMotor::stopMotor)
            .until(armFullyClosedTrigger);
  }

  public Command holdSetpointCommand(Translation2d setpoint) {
    return new ParallelCommandGroup(
            updateLastSetpointCommand(setpoint),
            moveToLengthCommand(setpoint),
            moveToAngleCommand(setpoint)
    );
  }

  private Command updateLastSetpointCommand(Translation2d setpoint){
    return new InstantCommand(()-> lastHeldSetpoint = setpoint);
  }

  public Command setAngleSpeed(double speed) {
    return new RunCommand(() -> angleMotor.set(speed / 100.0), this);
  }

  /**
   * when the motor stops, gravity slowly pulls the arm down, making the arm "fade" down
   *
   * @return the command
   */
  public Command fadeArmCommand(){
    return setAngleSpeed(0);
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
                        + kG_LENGTH * Math.sin(Units.degreesToRadians(getArmAngle()))
                        + kV_LENGTH * state.velocity;

                  lengthController.setReference(state.position, CANSparkMax.ControlType.kPosition,
                        0,
                        feedforward, SparkMaxPIDController.ArbFFUnits.kVoltage);
                }))
          .finallyDo((__) -> lengthMotor.stopMotor());
  }

  public Command moveToAngleCommand(Translation2d setpoint) {
    return Commands.runEnd(() -> {
      double setpointDeg = setpoint.getAngle().getDegrees() < 0? setpoint.getAngle().getDegrees() + 360 : setpoint.getAngle().getDegrees();
      double pid = kP_ANGLE * (setpointDeg - getArmAngle());
      double feedforward = kS_ANGLE * Math.signum(pid) + kG_ANGLE * setpoint.getAngle().getCos();

      if (getArmAngle() <= 80 || getArmAngle() >= 220) {
        DriverStation.reportError("odd arm angle reading while motor running! please check.", false);
        angleMotor.stopMotor();
      } else angleMotor.setVoltage(pid + feedforward);

    }, angleMotor::stopMotor, this);
  }


  /**
   * yes, this is a real command, it's meant to oscillate the arm up and down slightly,
   * to help the driver aim the arm better, and increase our success rate in cone placement to the high node
   * @param baseAngle the angle to oscillate from
   * @param magnitude the magnitude of the oscillations (degrees)
   * @return the command
   */
  public Command osscilateArmCommand(Translation2d baseAngle, double magnitude) {
    return Commands.repeatingSequence(
            moveToAngleCommand(baseAngle.rotateBy(Rotation2d.fromDegrees(-magnitude))).withTimeout(1),
            moveToAngleCommand(baseAngle.rotateBy(Rotation2d.fromDegrees(magnitude))).withTimeout(1)
    );
  }

  private boolean angleInRange(double angleA, double angleB) {
    double tolerance = 2; // degrees
    return Math.abs(angleA - angleB) < tolerance;
  }
  private boolean lengthInRange(double lengthA, double lengthB) {
    double tolerance = 1; // cm
    return Math.abs(lengthA - lengthB) < tolerance;
  }

  public boolean armAtSetpoint(Translation2d setpoint){
    return lengthInRange(setpoint.getNorm(), lengthEncoder.getPosition()) &&
            angleInRange(setpoint.getAngle().getDegrees(), getArmAngle());
  }

  public boolean armAtSetpoint(){
    return armAtSetpoint(lastHeldSetpoint);
  }

  public Command lockArmCommand(Trigger bbTrigger) {
    return resetLengthCommand()
          .andThen(moveToAngleCommand(LOCKED.setpoint)
          .alongWith(moveToLengthCommand(LOCKED.setpoint).unless(bbTrigger))).until(armLockedTrigger);
//          .until(armLockedTrigger).withName("lockArmCommand")
//            // checks is the arm is locked or is being locked and cancels if necessary.
//            .unless(armLockedTrigger.or(()->
//                    CommandScheduler.getInstance().requiring(this).getName().equals("lockArmCommand")));
  }

  public Command lockArmWithSetpoint(){
//    return holdSetpointCommand(LOCKED.setpoint).until(armLockedTrigger);
    return moveToAngleCommand(LOCKED.setpoint).withTimeout(1)
            .andThen(holdSetpointCommand(LOCKED.setpoint))
            .until(armLockedTrigger);
  }

  public Command stopTelescopeMotors() {
    return new InstantCommand(lengthMotor::stopMotor);
  }

  public Command toggleIdleModeCommand(){
    return new StartEndCommand(
            ()-> {
              angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
              angleFollowerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
            },
            ()-> {
              angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
              angleFollowerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            })
            .ignoringDisable(true);
  }

  @Override
  public void periodic() {
    if (armFullyClosedTrigger.getAsBoolean()) lengthEncoder.setPosition(MINIMAL_LENGTH_METERS);
  }

  private void initShuffleboardData(){
    armTab.addDouble("ArmLength", lengthEncoder::getPosition).withPosition(10, 0).withSize(4, 4)
            .withWidget("Number Slider").withProperties(Map.of("min", MINIMAL_LENGTH_METERS, "max", MAXIMAL_LENGTH_METERS));
    armTab.addDouble("Arm degrees", () -> -angleEncoder.getDistance()).withPosition(6, 0).withSize(4, 4)
            .withWidget("Simple Dial").withProperties(Map.of("min", 90, "max", 190));
    armTab.addBoolean("Fully closed", armFullyClosedTrigger).withPosition(6, 4)
            .withSize(4, 2);
    armTab.addBoolean("Arm locked", armFullyOpenedTrigger.and(armAngleClosedTrigger)).withPosition(8, 6)
            .withSize(4, 2);
  }
}
