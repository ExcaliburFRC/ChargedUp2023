package frc.robot.Subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ClawConstants.*;

public class Claw extends SubsystemBase {
    private final DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, FORWARD_CHANNEL, REVERSE_CHANNEL);

    private final DigitalInput beambreak = new DigitalInput(BEAMBREAK_CHANNEL);
    private final DigitalInput button = new DigitalInput(BUTTON_CHANNEL);

    private final Trigger isClawOpenedTrigger = new Trigger(() -> piston.get().equals(DoubleSolenoid.Value.kReverse));
    private final Trigger beambreakDetectedTrigger = new Trigger(() -> !beambreak.get());
    private final Trigger clawFullTrigger = new Trigger(() -> !button.get());

    public Claw() {
    }

    public Command manualCommand(BooleanSupplier toggle) {
        return togglePistonCommand();
    }

    public Command togglePistonCommand() {
        return new ConditionalCommand(
                new InstantCommand(() -> piston.set(DoubleSolenoid.Value.kForward), this),
                new InstantCommand(piston::toggle, this),
                () -> piston.get().equals(DoubleSolenoid.Value.kOff));
    }

    public Command autoClawCommand() {
        return new ConditionalCommand(
                autoCloseCommand(),
                openClawCommand(),
                clawFullTrigger);
    }

    private RunCommand autoCloseCommand() {
        return new RunCommand(
                () -> {
                    if (beambreakDetectedTrigger.getAsBoolean() && isClawOpenedTrigger.getAsBoolean())
                        piston.set(DoubleSolenoid.Value.kForward);
                });
    }

    private Command openClawCommand() {
        return new InstantCommand(() -> piston.set(DoubleSolenoid.Value.kReverse), this);
    }

    private Command closeClawCommand() {
        return new InstantCommand(() -> piston.set(DoubleSolenoid.Value.kForward), this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("claw full", clawFullTrigger, null);
        builder.addBooleanProperty("claw beam-break", beambreakDetectedTrigger, null);
    }
}
