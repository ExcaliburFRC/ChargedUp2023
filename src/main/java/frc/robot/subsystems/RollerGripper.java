package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.RollerGripperConstants.*;

public class RollerGripper extends SubsystemBase {
  private final CANSparkMax rightRoller = new CANSparkMax(RIGHT_ROLLER_MOTOR_ID, kBrushless);
  private final CANSparkMax leftRoller = new CANSparkMax(LEFT_ROLLER_MOTOR_ID, kBrushless);

  private final DigitalInput beambreak = new DigitalInput(INTAKE_BEAMBREAK);

  public final Trigger buttonTrigger = new Trigger(() -> !beambreak.get());

  public RollerGripper() {
    rightRoller.restoreFactoryDefaults();
    rightRoller.clearFaults();
    rightRoller.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftRoller.restoreFactoryDefaults();
    leftRoller.clearFaults();
    leftRoller.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftRoller.setInverted(false);
    rightRoller.setInverted(true);
  }

  private Command setRollerGripperMotor(double speed) {
    return new RunCommand(() -> {
            rightRoller.set(speed);
            leftRoller.set(speed);
          }, this);
  }

  public Command intakeCommand() {
    return Commands.runEnd(
                () -> {
                  rightRoller.set(0.6);
                  leftRoller.set(0.6);
                },
                () -> {
                  rightRoller.stopMotor();
                  leftRoller.stopMotor();
                },
                this)
          .until(buttonTrigger);
  }

  public Command ejectCommand() {
    return Commands.runEnd(
                () -> {
                  rightRoller.set(-0.05);
                  leftRoller.set(-0.05);
                },
                () -> {
                  rightRoller.stopMotor();
                  leftRoller.stopMotor();
                },
                this)
          .until(buttonTrigger.negate().debounce(0.2));
  }

  public Command releaseCommand(BooleanSupplier release) {
    return new RunCommand(() -> {
    }).until(release).andThen(ejectCommand());
  }

  public Command holdConeCommand() {
    return new ConditionalCommand(
          setRollerGripperMotor(0.05).until(()-> true),
          setRollerGripperMotor(0).until(()-> true),
          buttonTrigger)
          .repeatedly();
  }

  public Command manualCommand(BooleanSupplier intake, BooleanSupplier outtake, BooleanSupplier stop) {
    return Commands.runEnd(
          () -> {
            if (intake.getAsBoolean()) {
              rightRoller.set(0.6);
              leftRoller.set(0.6);
            }
            if (outtake.getAsBoolean()) {
              rightRoller.set(-0.1);
              leftRoller.set(-0.1);
            }
            if (stop.getAsBoolean()) {
              rightRoller.set(0);
              leftRoller.set(0);
            }
          },
          () -> {
            rightRoller.set(0);
            leftRoller.set(0);
          },
          this
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("rollerGripper button", buttonTrigger.getAsBoolean());
  }
}
