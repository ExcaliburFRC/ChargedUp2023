package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ClawConstants.*;

public class Claw extends SubsystemBase {
    private final DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, FORWARD_CHANNEL, REVERSE_CHANNEL);

    private final DigitalInput beambreak = new DigitalInput(BEAMBREAK_CHANNEL);

    public final Trigger isClawOpenedTrigger = new Trigger(() -> piston.get().equals(DoubleSolenoid.Value.kForward  ));
    private final Trigger beambreakDetectedTrigger = new Trigger(() -> !beambreak.get()).debounce(0.15);

    public Claw() {
        piston.set(DoubleSolenoid.Value.kOff);
    }

    public Command autoCloseCommand() {
        return new RunCommand(()->{}, this).until(beambreakDetectedTrigger)
              .andThen(closeClawCommand());
    }

    public Command openClawCommand() {
        return new InstantCommand(() -> piston.set(DoubleSolenoid.Value.kForward), this);
    }

    public Command closeClawCommand() {
        return new InstantCommand(() -> piston.set(DoubleSolenoid.Value.kReverse), this);
    }

    public Command releaseCommand(Trigger release){
        return new RunCommand(()-> {}).until(release).andThen(openClawCommand());
    }

    public Command toggleClawCommand(BooleanSupplier toggle){
        return new ConditionalCommand(
              openClawCommand(),
              closeClawCommand(),
              toggle
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Subsystem");
        builder.addBooleanProperty("claw beam-break", beambreakDetectedTrigger, null);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("bb", beambreakDetectedTrigger.getAsBoolean());
    }
}
