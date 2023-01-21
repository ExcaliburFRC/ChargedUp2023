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
    private final Trigger isClawOpened = new Trigger(()-> piston.get().equals(DoubleSolenoid.Value.kReverse));
    private final DigitalInput beambreak = new DigitalInput(BEAMBREAK_CHANNEL);
    private final Trigger bbTrigger = new Trigger(beambreak::get);
    private final DigitalInput button = new DigitalInput(BUTTON_CHANEL);
    private final Trigger clawFull = new Trigger(()-> !button.get());

    public Claw() {
    }

    public Command manualCommand(BooleanSupplier toOpen) {
        return new RunCommand(() -> {
            if (toOpen.getAsBoolean()) {
                if (piston.get().equals(DoubleSolenoid.Value.kReverse))
                    piston.set(DoubleSolenoid.Value.kForward);
            }
        }, this);
    }

    public Command toggleCommand() {
        return new ConditionalCommand(
                new InstantCommand(() -> piston.set(DoubleSolenoid.Value.kForward)),
                new InstantCommand(piston::toggle),
                () -> piston.get().equals(DoubleSolenoid.Value.kOff));
    }

    public Command autoClawCommand() {
        return Commands.run(
                () -> {
                    if (bbTrigger.getAsBoolean() && isClawOpened.getAsBoolean()) {
                        piston.set(DoubleSolenoid.Value.kForward);
                    }
                },
                this);
    }

    public Command openClawCommand(){
        return new InstantCommand(()-> piston.set(DoubleSolenoid.Value.kReverse), this);
    }

    public Command closeClawCommand(){
        return new InstantCommand(()-> piston.set(DoubleSolenoid.Value.kForward), this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("claw full", clawFull, null);
    }
}
