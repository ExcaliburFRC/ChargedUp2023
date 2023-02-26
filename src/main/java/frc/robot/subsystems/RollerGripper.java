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

  private final DigitalInput button = new DigitalInput(BUTTON_CHANNEL);

  public final Trigger buttonTrigger = new Trigger(()-> !button.get());

  public RollerGripper(){
    rightRoller.restoreFactoryDefaults();
    rightRoller.clearFaults();
    rightRoller.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftRoller.restoreFactoryDefaults();
    leftRoller.clearFaults();
    leftRoller.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public Command intakeCommand(){
    return Commands.runEnd(
          ()-> {
            rightRoller.set(0.3);
            leftRoller.set(0.3);
            },
                () -> {
            rightRoller.stopMotor();
            leftRoller.stopMotor();
            },
                this)
          .until(buttonTrigger);
  }

  public Command ejectCommand(){
    return Commands.runEnd(
          ()-> {
            rightRoller.set(-0.2);
            leftRoller.set(-0.2);
          },
                () -> {
            rightRoller.stopMotor();
            leftRoller.stopMotor();
                },
                this)
          .until(buttonTrigger.negate().debounce(0.2));
  }

  public Command releaseCommand(BooleanSupplier release){
    return new RunCommand(()-> {}).until(release).andThen(ejectCommand());
  }

  public Command manualCommand(BooleanSupplier intake, BooleanSupplier outtake, BooleanSupplier stop){
    return new RunCommand(
          ()-> {
            if (intake.getAsBoolean()) {
              rightRoller.set(0.2);
              leftRoller.set(0.2);
            }
            if (outtake.getAsBoolean()) {
              rightRoller.set(-0.2);
              leftRoller.set(-0.2);
            }
            if (stop.getAsBoolean()) {
              rightRoller.set(0);
              leftRoller.set(0);
            }
          }
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("rollerGripper button", buttonTrigger.getAsBoolean());
  }
}
