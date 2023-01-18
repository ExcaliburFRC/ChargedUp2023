package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ClawConstants.*;

public class Claw extends SubsystemBase {
  private final DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, FORWARD_CHANNEL, REVERSE_CHANNEL);

  public Claw() {
  }

  public Command manualCommand(BooleanSupplier toOpen) {
    return new RunCommand(() -> {
      if (toOpen.getAsBoolean()) {
        if (piston.get().equals(DoubleSolenoid.Value.kReverse))
          piston.set(DoubleSolenoid.Value.kForward);
      }
    }, this );
  }
}
