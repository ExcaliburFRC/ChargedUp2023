package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;

import static frc.robot.Constants.LedsConstants.Colors.OFF;
import static frc.robot.Constants.LedsConstants.*;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds = new AddressableLED(LEDS_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LENGTH);
    private static LEDs instance = null;

    private Color[] currentColors = new Color[LENGTH];

    private double brightness = 1;

    private int tailIndex = 0;


    public double getBrightness() {
        return brightness;
    }

    public void setBrightness(double brightness) {
        this.brightness = brightness;
    }

    private LEDs() {
        leds.setLength(LENGTH);
        leds.start();
    }

    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }
        return instance;
    }

    private void setLedColor(Color[] colors) {
        this.currentColors = colors;
        for (int i = 0; i < colors.length; i++) {
            buffer.setLED(i, colors[i]);
        }

        // debug for console
        if (!Robot.isRobotReal) {
            for (int i = 0; i < LENGTH; i++) {
                System.out.print("[]" + consoleColor.get(currentColors[i]));
            }
            System.out.println();
        }

        leds.setData(buffer);
    }

    public Color applyBrightness(double brightness, Color color) {
        this.brightness = brightness;
        return new Color(color.red * this.brightness, color.green * this.brightness, color.blue * this.brightness);
    }

    public Color applyBrightness(Color color) {
        return applyBrightness(this.brightness, color);
    }

    public void restoreLEDs() {
        CommandScheduler.getInstance().cancel(
                CommandScheduler.getInstance().requiring(this));
    }

    public enum LEDPattern {
        OFF,
        RAINBOW,
        SOLID,
        ALTERNATING,
        TRAIN_CIRCLE,
        TRAIN,
        RANDOM,
        BLINKING;
    }

    public Command applyPatternCommand(LEDPattern pattern, Color color) {
        return applyPatternCommand(pattern, color, OFF.color);
    }

    public Command applyPatternCommand(LEDPattern pattern, Color mainColor, Color accentColor) {
        Command command = new InstantCommand();
        Color[] colors = new Color[LENGTH];
        int trainLength = (int) MathUtil.clamp(LENGTH * 0.25, 1.0, LENGTH / 2.0);
        final AtomicBoolean invert = new AtomicBoolean(false);

        switch (pattern) {
            case OFF:
                Arrays.fill(colors, OFF.color);
                command = new RunCommand(() -> setLedColor(colors), this).withName("OFF");
                break;

            case SOLID:
                Arrays.fill(colors, mainColor);
                command = new RunCommand(() -> setLedColor(colors), this).withName("SOLID: " + mainColor.toString());
                break;

            case ALTERNATING:
                for (int i = 0; i < LENGTH; i++) {
                    colors[i] = mainColor;
                    colors[i + 1] = accentColor;
                    i++;
                }
                command = new RunCommand(() -> setLedColor(colors), this)
                        .withName("ALTERNATING, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case RANDOM:
                command = new InstantCommand(() -> {
                    Arrays.fill(colors, new Color(Math.random() * 255, Math.random() * 255, Math.random() * 255));
                    setLedColor(colors);
                }, this)
                        // adds a delay to the command iteration
                        .andThen(new WaitCommand(0.1)).repeatedly()
                        .withName("RANDOM");
                break;

            case BLINKING:
                command = Commands.repeatingSequence(
                        new InstantCommand(() -> {
                            Arrays.fill(colors, mainColor);
                            setLedColor(colors);
                        }, this),
                        new WaitCommand(0.5),
                        new InstantCommand(() -> {
                            Arrays.fill(colors, accentColor);
                            setLedColor(colors);
                        }, this),
                        new WaitCommand(0.5)
                ).withName("BLINKING, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case TRAIN:
                command = new InstantCommand(() -> {
                    Arrays.fill(colors, mainColor);
                    for (int j = 0; j < trainLength; j++) colors[MathUtil.clamp(j + tailIndex - 1, 0 , LENGTH)] = accentColor;
                    this.tailIndex = invert.get() ? this.tailIndex - 1 : this.tailIndex + 1;
                    if (this.tailIndex == LENGTH - trainLength || this.tailIndex == 0) invert.set(!invert.get());
                    setLedColor(colors);
                }, this)
                        .andThen(new WaitCommand(0.05)).repeatedly()
                        .withName("TRAIN_BACK_AND_FOURTH, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case TRAIN_CIRCLE:
                command = new InstantCommand(() -> {
                    Arrays.fill(colors, mainColor);
                    for (int j = 0; j < trainLength; j++) colors[stayInBounds(j + tailIndex, LENGTH)] = accentColor;
                    tailIndex ++;
                    if (tailIndex == LENGTH) tailIndex = 0;
                    setLedColor(colors);
                }, this)
                        .andThen(new WaitCommand(0.01)).repeatedly()
                        .withName("TRAIN_CIRCLE, main: " + mainColor.toString() + ", accent: " + accentColor.toString());

            default:
                break;
        }

        return command.ignoringDisable(true).finallyDo((__) -> restoreLEDs());
    }

    private int findHeadIndex(Color[] colors, Color trainColor) {
        for (int i = colors.length - 1; i >= 0; i--) {
            if (colors[i].equals(trainColor)) return i;
        }
        return -1;
    }

    private int stayInBounds(int value, int length) {
        if (value >= length) return stayInBounds(value - length, length);
        if (value < 0) return stayInBounds(value + length, length);
        return value;
    }
}