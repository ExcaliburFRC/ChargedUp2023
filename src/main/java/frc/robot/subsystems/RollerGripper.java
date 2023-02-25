package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.ClawConstants.*;

public class RollerGripper extends SubsystemBase {
  private final CANSparkMax roller = new CANSparkMax(ROLLER_ID, kBrushless);

  private final DigitalInput button = new DigitalInput(BEAMBREAK_CHANNEL);

  public final Trigger buttonTrigger = new Trigger(()-> !button.get());

  public RollerGripper(){
    roller.restoreFactoryDefaults();
    roller.clearFaults();
    roller.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public Command intakeCommand(){
    return Commands.runEnd(
          ()-> roller.set(0.3),
          roller::stopMotor,
                this)
          .until(buttonTrigger);
  }

  public Command ejectCommand(){
    return Commands.runEnd(
          ()-> roller.set(-0.2),
          roller::stopMotor,
                this)
          .withTimeout(0.2);
  }

  public Command releaseCommand(BooleanSupplier release){
    return new RunCommand(()-> {}).until(release).andThen(ejectCommand());
  }

  public Command manualCommand(BooleanSupplier intake, BooleanSupplier outtake, BooleanSupplier stop){
    return new RunCommand(
          ()-> {
            if (intake.getAsBoolean()) roller.set(0.2);
            if (outtake.getAsBoolean()) roller.set(-0.2);
            if (stop.getAsBoolean()) roller.set(0);
          }
    );
  }
}
