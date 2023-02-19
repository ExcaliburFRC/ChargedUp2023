package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ClawConstants.*;

public class Claw extends SubsystemBase {
    private final DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, FORWARD_CHANNEL, REVERSE_CHANNEL);

    private final DigitalInput beambreak = new DigitalInput(BEAMBREAK_CHANNEL);

    private final Trigger isClawOpenedTrigger = new Trigger(() -> piston.get().equals(DoubleSolenoid.Value.kReverse));
    private final Trigger beambreakDetectedTrigger = new Trigger(() -> !beambreak.get());

    public Claw() {
        piston.set(DoubleSolenoid.Value.kOff);
    }

    public Command autoCloseCommand() {
        return new RunCommand(
                () -> {
                    if (beambreakDetectedTrigger.getAsBoolean() && isClawOpenedTrigger.getAsBoolean())
                        piston.set(DoubleSolenoid.Value.kForward);
                });
    }

    public Command openClawCommand() {
        return new InstantCommand(() -> piston.set(DoubleSolenoid.Value.kReverse), this);
    }

    public Command closeClawCommand() {
        return new InstantCommand(() -> piston.set(DoubleSolenoid.Value.kForward), this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("claw beam-break", beambreakDetectedTrigger, null);
    }
}
