package frc.robot.subsystems;

import static frc.robot.Constants.LedsConstants.LEDS_PORT;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.*;

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
        ORANGE(0.65),
        GREEN(0.73),
        YELLOW(0.67),
        WHITE(0.93),
        PURPLE(0.91),
        PINK(0.57),
        GOLD(0.67),
        GRAY(0.95),
        BLACK(0.99),
        BLUE_GREEN(0.79),
        RAINBOW(-0.97),
        LIGHT_CHASE_BLUE(-0.29), // TODO: check if works, if so: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
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
        return LedMode.LIGHT_CHASE_BLUE;
    }

    public Command setColorCommand(LedMode color) {
        return new RunCommand(() -> leds.setSpeed(color.dutyCycle), this);
    }

    public Command restoreDefualtColorCommand(){
        return new InstantCommand(()-> CommandScheduler.getInstance().requiring(this).cancel());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SendableRegistry.remove(leds);
    }
}