package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ClawConstants.*;

public class Claw extends SubsystemBase {
  private final DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, FORWARD_CHANNEL, REVERSE_CHANNEL);

  public Claw() {}

  public Command manualCommand(BooleanSupplier toOpen) {
    return new RunCommand(() -> {
      if (toOpen.getAsBoolean()) {
        if (piston.get().equals(DoubleSolenoid.Value.kReverse))
          piston.set(DoubleSolenoid.Value.kForward);
      }
    }, this);
  }

  public Command toggleCommand(){
    return new ConditionalCommand(
          new InstantCommand(()-> piston.set(DoubleSolenoid.Value.kForward)),
          new InstantCommand(piston::toggle),
          ()-> piston.get().equals(DoubleSolenoid.Value.kOff));
  }
}
