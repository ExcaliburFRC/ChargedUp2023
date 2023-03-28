package frc.robot.subsystems;

import static frc.robot.Constants.LedsConstants.LENGTH;
import static frc.robot.Constants.LedsConstants.LEDS_PORT;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.*;
import static frc.robot.Constants.LedsConstants.Colors.*;
import frc.robot.utility.Color;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds = new AddressableLED(LEDS_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LENGTH);

    private double brightness = 1;

    public double getBrightness() {
        return brightness;
    }

    public void setBrightness(double brightness) {
        this.brightness = brightness;
    }

    public LEDs() {
        leds.setData(buffer);
        leds.setLength(LENGTH);
        leds.start();
    }

    private void setLedColor(Color[] colors){
        for (int i = 0; i < colors.length; i++) {
            buffer.setLED(i, Color.balance(colors[i]));
        }
        leds.setData(buffer);
    }

    public Color applyBrightness(double brightness, Color color){
        this.brightness = brightness;
        return new Color(color.red * this.brightness, color.green * this.brightness, color.blue * this.brightness);
    }

    public Color applyBrightness(Color color){
        return applyBrightness(this.brightness, color);
    }

    public void restoreLEDs(){
            CommandScheduler.getInstance().cancel(
                    CommandScheduler.getInstance().requiring(this));
    }

    public enum LEDPattern{
        OFF,
        RAINBOW,
        SOLID,
        ALTERNATING,
        TRAIN_CIRCLE,
        TRAIN_BACK_AND_FOURTH,
        RANDOM,
        BLINKING;
    }

    public Command applyPatternCommand(LEDPattern pattern, Color color){
        return applyPatternCommand(pattern, color, OFF.color);
    }

    public Command applyPatternCommand(LEDPattern pattern, Color mainColor, Color accentColor){
        Command command = new InstantCommand();
        Color[] colors = new Color[LENGTH];
        int trainLength = (int) MathUtil.clamp(LENGTH / 10.0, 1.0, LENGTH / 2.0);

        switch (pattern){
            case OFF:
                Arrays.fill(colors, OFF.color);
                command = new RunCommand(()-> setLedColor(colors), this).withName("OFF");
                break;

            case SOLID:
                Arrays.fill(colors, mainColor);
                command = new RunCommand(()-> setLedColor(colors), this).withName("SOLID: " + mainColor.toString());
                break;

            case ALTERNATING:
                for (int i = 0; i < LENGTH; i++) {
                    colors[i] = mainColor;
                    colors[i + 1] = accentColor;
                    i ++;
                }
                command = new RunCommand(()-> setLedColor(colors), this)
                        .withName("ALTERNATING, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case TRAIN_CIRCLE:
                command = new InstantCommand(()-> {
                    int headIndex = findHeadIndex(colors, accentColor) == -1? 0 : findHeadIndex(colors, accentColor);
                    Arrays.fill(colors, mainColor);
                    for (int i = 0; i < trainLength; i++) {
                        colors[stayInBounds(i + headIndex, LENGTH)] = accentColor;
                    }
                    setLedColor(colors);
                }, this)
                        // adds a delay to the command iteration
                .andThen(new WaitCommand(0.1)).repeatedly()
                        .withName("TRAIN_CIRCLE, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case RANDOM:
                command = new InstantCommand(()-> {
                    Arrays.fill(colors, new Color(Math.random() * 255, Math.random() * 255, Math.random() * 255));
                    setLedColor(colors);
                }, this)
                        // adds a delay to the command iteration
                        .andThen(new WaitCommand(0.1)).repeatedly()
                        .withName("RANDOM");
                break;

            case BLINKING:
                command = Commands.repeatingSequence(
                        new InstantCommand(()-> setLedColor(new Color[]{mainColor})),
                        new WaitCommand(0.5),
                        new InstantCommand(()-> setLedColor(new Color[]{accentColor})),
                        new WaitCommand(0.5)
                ).withName("BLINKING, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case TRAIN_BACK_AND_FOURTH:
                final AtomicBoolean invert = new AtomicBoolean(false);
                command = new RunCommand(()-> {
                    for (int i = 0; i < LENGTH; i++) {
                        Arrays.fill(colors, mainColor);
                        for (int j = 0; j < trainLength; j++) {
                            colors[MathUtil.clamp(invert.get()? LENGTH - 1 - j : j, 0, LENGTH - 1)] = accentColor;
                        }
                        setLedColor(colors);
                    }
                    invert.set(!invert.get());
                }, this).repeatedly()
                        .withName("TRAIN_BACK_AND_FOURTH, main: " + mainColor.toString() + ", accent: " + accentColor.toString());


            default:
                break;
        }

        return command.ignoringDisable(true).finallyDo((__)-> restoreLEDs());
    }

    private int findHeadIndex(Color[] colors, Color trainColor){
        for (int i = colors.length - 1; i > 0; i--) {
            if (colors[i].equals(trainColor)) return i;
        }
        return -1;
    }

    private int stayInBounds(int value, int length){
        if (value > length) return stayInBounds(value - length, length);
        if (value < 0) return stayInBounds(value + length, length);
        return value;
    }
}