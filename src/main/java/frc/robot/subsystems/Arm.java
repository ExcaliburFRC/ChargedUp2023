package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.Setpoints.LOCKED;

public class Arm extends SubsystemBase {
  private final CANSparkMax angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax angleFollowerMotor = new CANSparkMax(ANGLE_FOLLOWER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax lengthMotor = new CANSparkMax(LENGTH_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final RelativeEncoder lengthEncoder = lengthMotor.getEncoder();
  private final RelativeEncoder angleRelativeEncoder = angleMotor.getEncoder();

  private final DutyCycleEncoder angleEncoder = new DutyCycleEncoder(ABS_ANGLE_ENCODER_PORT);

  private final DigitalInput lowerLimitSwitch = new DigitalInput(CLOSED_LIMIT_SWITCH_ID);

  public final Trigger armFullyClosedTrigger = new Trigger(() -> !lowerLimitSwitch.get());
  public final Trigger armFullyClosedTriggerEncoder = new Trigger(() -> lengthEncoder.getPosition() <= MINIMAL_LENGTH_METERS);
  public final Trigger armFullyOpenedTrigger = new Trigger(() -> lengthEncoder.getPosition() >= LOCKED_LENGTH_METERS - 0.02);

  public final Trigger armAngleClosedTrigger = new Trigger(() -> getArmAngle() <= 95);

  public final Trigger armLockedTrigger = armAngleClosedTrigger.and(armFullyOpenedTrigger);

  private final SparkMaxPIDController lengthController = lengthMotor.getPIDController();
  private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0, 0), new TrapezoidProfile.State());
  private final Timer trapozoidTimer = new Timer();

  public static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

  private AtomicReference<Translation2d> lastHeldSetpoint = new AtomicReference<>(new Translation2d());

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

    angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 200f);
    angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    angleFollowerMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 200f);
    angleFollowerMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    lengthController.setP(kP_LENGTH);
    lengthController.setI(0);
    lengthController.setD(kD_LENGTH);

    angleMotor.setOpenLoopRampRate(1.5);

    lengthEncoder.setPosition(LOCKED_LENGTH_METERS);

    initShuffleboardData();
    setDefaultCommand(setAngleSpeed(0));
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
    }, this);
  }
  public Command resetLengthEncoderCommand() {
    return Commands.runEnd(
        () -> lengthMotor.set(-0.75),
        lengthMotor::stopMotor, this)
      .until(armFullyClosedTriggerEncoder);
  }
  public Command resetLengthCommand() {
    return Commands.runEnd(
          () -> lengthMotor.set(-0.75),
          lengthMotor::stopMotor, this)
            .until(()-> !lowerLimitSwitch.get());
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


  public Command holdSetpointCommand(Translation2d setpoint) {
    return new ParallelCommandGroup(
            updateLastSetpointCommand(setpoint),
            moveToLengthCommand(setpoint),
            moveToAngleCommand(setpoint)
    );
  }

  private Command updateLastSetpointCommand(Translation2d setpoint){
    return new InstantCommand(()-> lastHeldSetpoint.set(setpoint));
  }

  public Command setAngleSpeed(double speed) {
    return new RunCommand(() -> angleMotor.set(speed / 100.0), this);
  }

  public Command moveToLengthCommand(Translation2d setpoint) {
    return new FunctionalCommand(
            ()-> {
              trapozoidTimer.restart();
              trapezoidProfile = new TrapezoidProfile(
                      new TrapezoidProfile.Constraints(kMaxLinearVelocity, kMaxLinearAcceleration),
                      new TrapezoidProfile.State(setpoint.getNorm(), 0),
                      new TrapezoidProfile.State(lengthEncoder.getPosition(), lengthEncoder.getVelocity()));
            },
            ()-> {
              TrapezoidProfile.State state = trapezoidProfile.calculate(trapozoidTimer.get());
              double feedforward = kS_LENGTH * Math.signum(state.velocity)
                        + kG_LENGTH * Math.sin(Units.degreesToRadians(getArmAngle()))
                        + kV_LENGTH * state.velocity;

              lengthController.setReference(
                      state.position, CANSparkMax.ControlType.kPosition, 0,
                      feedforward, SparkMaxPIDController.ArbFFUnits.kVoltage);
                },
            (__)-> trapozoidTimer.stop(),
            ()-> trapozoidTimer.hasElapsed(trapezoidProfile.totalTime()));
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
   * @param magnitude the magnitude of the oscillations (degrees)
   * @return the command
   */
  public Command osscilateArmCommand(double magnitude) {
    return Commands.repeatingSequence(
            setAngleSpeed(magnitude).withTimeout(1),
            setAngleSpeed(0).withTimeout(1)
//            moveToAngleCommand(baseAngle.rotateBy(Rotation2d.fromDegrees(-magnitude))).withTimeout(3),
//            moveToAngleCommand(baseAngle.rotateBy(Rotation2d.fromDegrees(magnitude))).withTimeout(3)
    );
  }

  private boolean angleInRange(double angleA, double angleB) {
    double tolerance = 3; // degrees
    return Math.abs(angleA - angleB) < tolerance;
  }
  private boolean lengthInRange(double lengthA, double lengthB) {
    double tolerance = 1; // cm
    return Math.abs(lengthA - lengthB) < (tolerance / 100.0);
  }

  public boolean armAtSetpoint(Translation2d setpoint){
    return lengthInRange(setpoint.getNorm(), lengthEncoder.getPosition()) &&
            angleInRange(setpoint.getAngle().getDegrees(), getArmAngle());
  }

  public boolean armAtSetpoint(){
    return armAtSetpoint(lastHeldSetpoint.get());
  }

  public Command lockArmCommand(Setpoints setpoint) {
    return new SequentialCommandGroup(
            resetLengthCommand(),
            new WaitCommand(0.15),
            moveToAngleCommand(LOCKED.setpoint).alongWith(moveToLengthCommand(setpoint.setpoint)).until(armLockedTrigger),
            stopTelescopeMotor());
  }

  public Command forceLockArm(){
    return new SequentialCommandGroup(
            resetLengthCommand(),
            new WaitCommand(0.15),
            holdSetpointCommand(LOCKED.setpoint).until(this::armAtSetpoint))
            .finallyDo((__)-> stopTelescopeMotor().schedule());
  }

  public Command lockArmWithSetpoint(){
    return moveToAngleCommand(LOCKED.setpoint).withTimeout(1)
            .andThen(holdSetpointCommand(LOCKED.setpoint))
            .until(armLockedTrigger).finallyDo((__)-> stopTelescopeMotor().schedule());
  }

  public Command stopTelescopeMotor() {
    return new InstantCommand(lengthMotor::stopMotor);
  }

  public Command toggleIdleModeCommand(){
    return new StartEndCommand(
            ()-> {
              angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
              angleFollowerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
              lengthMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
            },
            ()-> {
              angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
              angleFollowerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
              lengthMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            })
            .ignoringDisable(true);
  }

  @Override
  public void periodic() {
    if (armFullyClosedTrigger.getAsBoolean()) lengthEncoder.setPosition(MINIMAL_LENGTH_METERS);
    angleRelativeEncoder.setPosition(getArmAngle());
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
