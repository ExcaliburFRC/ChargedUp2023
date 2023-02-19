package frc.robot.subsystems;

import static frc.robot.Constants.LedsConstants.LEDS_PORT;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final PWM leds = new PWM(LEDS_PORT);

    public LEDs() {
        SendableRegistry.remove(leds);
        SendableRegistry.remove(this);
    }

    @SuppressWarnings("unused")
    public enum LedMode {
        BLUE(0.87),
        RED(0.61),
        GREEN(0.73),
        YELLOW(0.67),
        RAINBOW(-0.97),
        WHITE(0.93),
        VIOLET(0.91),
        PINK(0.57),
        GOLD(0.67),
        BLACK(0.99),
        OFF(0.99);

        LedMode(double c) {
            dutyCycle = c;
        }

        public final double dutyCycle;
    }

    public LedMode getAlliance() {
        switch (DriverStation.getAlliance()) {
            case Blue:
                return LedMode.BLUE;
            case Red:
                return LedMode.RED;
        }
        return LedMode.GOLD;
    }

    public Command setColorCommand(LedMode color) {
        return new RunCommand(() -> leds.setSpeed(color.dutyCycle), this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SendableRegistry.remove(leds);
    }
}