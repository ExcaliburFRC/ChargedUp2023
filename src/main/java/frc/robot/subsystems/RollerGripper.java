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

  public final Trigger beambreakTrigger = new Trigger(() -> !beambreak.get());

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

  /**
   * sets the roller gripper motors speeds
   * @param speed the speed the motors spin at
   * @return the command
   */
  private Command setRollerGripperMotor(double speed) {
    return new RunCommand(() -> {
            rightRoller.set(speed);
            leftRoller.set(speed);
          }, this);
  }

  /**
   * intakeCommand
   * <p><b> noInit </b>starts the motors <br>
   * <b>noEnd </b>stops the motors</p>
   * <b>ends when the button senses that a cone was collected into the roller gripper </b>
   * @return the command
   */
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
          .until(beambreakTrigger);
  }

  /**
   * ejectCommand
   * <p><b> noInit </b>starts spinning the motors outwards<br>
   * <b>noEnd </b>stops the motors</p>
   * ends when the button senses that a cone is not longer in the roller gripper
   * @return the command
   */
  public Command ejectCommand() {
    return Commands.runEnd(
                () -> {
                  rightRoller.set(-0.025);
                  leftRoller.set(-0.025);
                },
                () -> {
                  rightRoller.stopMotor();
                  leftRoller.stopMotor();
                },
                this)
          .until(beambreakTrigger.negate().debounce(0.2));
  }

  /**
   * waits until the driver wants to release the cone and then returns the eject command
   * @param release the button that needs to be pressed in order to eject
   * @return the eject command after the button was pressed
   */
  public Command releaseCommand(BooleanSupplier release) {
    return new RunCommand(() -> {
    }).until(release).andThen(ejectCommand());
  }

  /**
   * applies a small force to the roller gripper in order to hold it in place
   * @return the command
   */
  public Command holdConeCommand() {
    return new ConditionalCommand(
          setRollerGripperMotor(0.05).until(()-> true),
          setRollerGripperMotor(0).until(()-> true),
          beambreakTrigger)
          .repeatedly();
  }

  /**
   * manual command the allows full manual control of the system
   * @param intake whether the system should currently intake
   * @param outtake whether the system should currently eject
   * @param stop whether the system should currently be stopped
   * @return the command
   */
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
    SmartDashboard.putBoolean("rollerGripper button", beambreakTrigger.getAsBoolean());
  }
}
