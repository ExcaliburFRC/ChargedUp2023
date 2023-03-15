package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utility.Limelight;

import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, kBrushless);

  private final DoubleSolenoid intakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, INTAKE_FWD_CHANNEL, INTAKE_REV_CHANNEL);
  private final DoubleSolenoid ejectPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, EJECT_FWD_CHANNEL, EJECT_REV_CHANNEL);

  private final PIDController pidController = new PIDController(kP, 0, 0);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

  public final Trigger isAtTargetVelocity = new Trigger(
        () -> Math.abs(pidController.getPositionError()) < TOLERANCE).debounce(0.1);
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    intakeMotor.clearFaults();
    intakeEncoder.setAverageDepth(8);
    intakeEncoder.setMeasurementPeriod(32);
//        intakeMotor.getFault()
  }

  public Command openPistonCommand() {
    return new InstantCommand(() -> {
      intakePiston.set(DoubleSolenoid.Value.kForward);
    }, this);
  }

  public Command closePistonCommand() {
    return new InstantCommand(() -> {
      intakePiston.set(DoubleSolenoid.Value.kReverse);
    }, this);
  }

  /**
   * intakeCommand
   * <p><b> noInit </b>opens the pistons and starts the motor <br>
   * <b>noEnd </b>closes the pistons and stops the motor</p>
   * <b>Warning the driver must stop the command, it will NOT end automatically</b>
   * @param intakeSpeed the motor percentage to intake in
   * @return the intakeCommand
   */
  public Command intakeCommand(double intakeSpeed) {
    return new StartEndCommand(
          () -> {
            intakePiston.set(DoubleSolenoid.Value.kForward);
            intakeMotor.set(intakeSpeed);
//            Shuffleboard.selectTab("intakeCamera");
          },
          () -> {
            intakePiston.set(DoubleSolenoid.Value.kReverse);
            intakeMotor.stopMotor();
//            Shuffleboard.selectTab("Swerve");
          }, this).andThen();
  }

  private Command pushCubeCommand() {
    return new InstantCommand((() -> ejectPiston.set(DoubleSolenoid.Value.kForward)));
  }

  private Command retractPistonCommand() {
    return new InstantCommand((() -> ejectPiston.set(DoubleSolenoid.Value.kReverse)));
  }

  @Deprecated
  public Command shootCubeCommand(int height, DoubleSupplier offset) {
    switch (height) {
      case 1:
        return Commands.repeatingSequence(
                    Commands.runEnd(() -> intakeMotor.set(-0.2), intakeMotor::stopMotor, this).withTimeout(0.3), pulseMotorCommand())
              .alongWith(pushCubeCommand())
              .finallyDo((__) -> ejectPiston.set(DoubleSolenoid.Value.kReverse));
      case 2:
        return Commands.runEnd(() -> intakeMotor.set(-0.4 + offset.getAsDouble()), intakeMotor::stopMotor, this)
              .alongWith(new WaitCommand(0.2).andThen(pushCubeCommand()))
              .finallyDo((__) -> ejectPiston.set(DoubleSolenoid.Value.kReverse));
      case 3:
        return Commands.runEnd(() -> intakeMotor.set(-0.7 + offset.getAsDouble()), intakeMotor::stopMotor, this)
              .alongWith(new WaitCommand(0.45).andThen(pushCubeCommand()))
              .finallyDo((__) -> ejectPiston.set(DoubleSolenoid.Value.kReverse));
      default:
        return new InstantCommand(() -> {
        });
    }
  }

  /**
   * ShootCubeCommand.
   *<p> for low shooting input rpm = 0 </p>
   * @param rpm the desired rpm to shoot at
   */
  public Command shootCubeCommand(double rpm) {
    return new ConditionalCommand(
          // pid shooter
          this.run(
                () -> {
                  double pid = pidController.calculate(intakeEncoder.getVelocity(), rpm);
                  double ff = feedforward.calculate(rpm);

                  intakeMotor.setVoltage(pid + ff);
                  SmartDashboard.putNumber("rpm", intakeEncoder.getVelocity());
                }).alongWith(new WaitUntilCommand(isAtTargetVelocity).andThen(pushCubeCommand()))
                .finallyDo((__) -> {
                  intakeMotor.stopMotor();
                  ejectPiston.set(DoubleSolenoid.Value.kReverse);
                }),
          // pulse shooter to low
          shootCubeToLowCommand(),
          () -> rpm != 0
    );
  }

  /**
   * shootCubeToLowCommand
   * shoots the cube to the low height using motor pulses
   * instead of pid based velocity controlled
   * @return the command
   */
  public Command shootCubeToLowCommand() {
    return Commands.repeatingSequence(
                Commands.runEnd(() -> intakeMotor.setVoltage(-1), intakeMotor::stopMotor, this)
                      .withTimeout(0.2), //-1, 0.2
                pulseMotorCommand())
          .alongWith(pushCubeCommand())
          .finallyDo((__) -> ejectPiston.set(DoubleSolenoid.Value.kReverse));
  }

  /**
   * pulses the motor for a set period of time (7 ms)
   * at 85% motor speed
   * @return
   */
  public Command pulseMotorCommand() { // -10, 0.07
    return Commands.runEnd(() -> intakeMotor.setVoltage(-10), intakeMotor::stopMotor).withTimeout(0.07);
  }

  public Command orientCubeCommand(){
    return this.runEnd(
          ()-> intakeMotor.set(0.4),
          intakeMotor::stopMotor
          );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("shooter current", intakeMotor::getOutputCurrent, null);
    builder.addDoubleProperty("shooter velocity", intakeEncoder::getVelocity, null);
    builder.addDoubleProperty("applied output", intakeMotor::getAppliedOutput, null);
  }
}
