package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private final Trigger armFullyOpenedTrigger = new Trigger(() -> lengthEncoder.getPosition() > 0.92);

  private final SparkMaxPIDController anglePIDController = angleMotor.getPIDController();

  public Arm() {
    angleFollowerMotor.restoreFactoryDefaults();
    angleMotor.restoreFactoryDefaults();
    lengthMotor.restoreFactoryDefaults();

    angleFollowerMotor.follow(angleMotor, false);
    angleMotor.setInverted(true);
    lengthMotor.setInverted(true);

    angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    angleFollowerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lengthMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    lengthEncoder.setPositionConversionFactor(ROT_TO_METER);
    lengthEncoder.setVelocityConversionFactor(RPM_TO_METER_PER_SEC);

//    angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 19.5f); // todo: fixxx
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

  /**
   * holds the arm in a given DutyCycle
   * @param dc the dc to hold the arm at
   * @param accel acceleration button
   * @param reduce deceleration button
   * @return the command
   */
  public Command holdArmCommand(double dc, BooleanSupplier accel, BooleanSupplier reduce ) {
    return moveToDutyCycleCommand(dc, accel, reduce);
  }

  /**
   * retracts the telescope until fully closed
   * ends when the armFullyClosedTrigger is true
   * @return the command
   */
  public Command retractTelescopeCommand(){
    return Commands.runEnd(()-> lengthMotor.set(-0.45), lengthMotor::stopMotor, this)
          .until(armFullyClosedTrigger);
  }

  /**
   * retracts the arm's angle until it's inside the frame perimeter
   * @return
   */
  private Command retractArmCommand(){
    return Commands.runEnd(()-> angleMotor.set(0.06), angleMotor::stopMotor).until(()-> armAngleClosed());
  }

  /**
   * @return whether the arm is inside the frame perimeter
   */
  private boolean armAngleClosed(){
    return Math.abs(270 - getArmDegrees()) < 10;
  }

  public Command lowerTelescopeCommand(){
    return new RunCommand(()-> lengthMotor.set(0.3), this)
          .until(()-> lengthInRange(1.58))
          .andThen(new InstantCommand(lengthMotor::stopMotor), new WaitUntilCommand(()-> false));
  }

  public Command defaultCommand(){
    return lowerTelescopeCommand()
          .alongWith(retractArmCommand())
          .andThen(new WaitUntilCommand(()-> false));
  }

  private Command moveToDutyCycleCommand(double dc, BooleanSupplier accel, BooleanSupplier reduce) {
    return Commands.runEnd(() -> {
            double a = accel.getAsBoolean() ? -0.075 : 0;
            double r = reduce.getAsBoolean() ? 0.075 : 0;
            angleMotor.set(dc + a + r);
          }, angleMotor::stopMotor,
          this);
  }

  public Command flipRopeCommand(){
    return new RunCommand(()-> lengthMotor.set(-0.3), this);
  }

  private Command moveToAngleCommand(double angle){
    double kp = 0.1;
    double kg = getArmDegrees() * 0;
    double ff = kg * Math.cos(angle);
    return Commands.runEnd(
          ()-> {
            angleMotor.set(ff + kp * (angle - getArmDegrees()));
            System.out.println("ff: " + ff);
            System.out.println("output: " + kp * (angle - getArmDegrees()));
          },
          angleMotor::stopMotor
    );
  }

  private ParallelRaceGroup extendLengthCommand(double telescope) {
    return Commands.runEnd(() -> {
      if (telescope < lengthEncoder.getPosition())
        lengthMotor.set(-0.15);
      else lengthMotor.set(0.15);
    }, lengthMotor::stopMotor).until(() -> lengthInRange(telescope));
  }

  private boolean lengthInRange(double setpoint) {
    double tolorance = 0.0785; //TODO: find
    return Math.abs(setpoint - lengthEncoder.getPosition()) < tolorance;
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
