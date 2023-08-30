package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.utility.Color;
import frc.robot.utility.Colors;

import java.util.Arrays;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.LedsConstants.LEDS_PORT;
import static frc.robot.Constants.LedsConstants.LENGTH;
import static frc.robot.utility.Colors.OFF;
import static frc.robot.utility.Colors.TEAM_GOLD;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds = new AddressableLED(LEDS_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LENGTH);
    private static LEDs instance = null;
    private Random rnd = new Random();

    private Color[] currentColors = new Color[LENGTH];

    private int tailIndex = 0;
    private double offset = 0;

    private LEDs() {
        leds.setLength(LENGTH);
        leds.start();

        setDefaultCommand(applyPatternCommand(LEDPattern.TRAIN, getAllianceColor(), TEAM_GOLD.color));
    }

    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }
        return instance;
    }

    public Command applyPatternCommand(LEDPattern pattern, Color mainColor, Color accentColor) {
        Command command = new InstantCommand();
        Color[] colors = new Color[LENGTH];
        int trainLength = (int) MathUtil.clamp(LENGTH * 0.15, 1.0, LENGTH / 2.0);
        final AtomicBoolean invert = new AtomicBoolean(false);

        switch (pattern) {
            case OFF:
                Arrays.fill(colors, OFF.color);
                command = new RunCommand(() -> setLedStrip(colors), this).withName("OFF");
                break;

            case SOLID:
                Arrays.fill(colors, mainColor);
                command = new RunCommand(() -> setLedStrip(colors), this).withName("SOLID: " + mainColor.toString());
                break;

            case ALTERNATING_STATIC:
                for (int i = 0; i < LENGTH; i++) {
                    colors[i] = mainColor;
                    colors[i + 1] = accentColor;
                    i++;
                }
                command = new RunCommand(() -> setLedStrip(colors), this)
                        .withName("ALTERNATING, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case ALTERNATING_MOVING:
                AtomicReference<Color> mainAlternatingColor = new AtomicReference<>(mainColor);
                AtomicReference<Color> accentAlternatingColor = new AtomicReference<>(accentColor);
                AtomicReference<Color> tempAlternatingColor = new AtomicReference<>();

                command = this.runOnce(()-> {
                            for (int i = 0; i < LENGTH; i++) {
                                colors[i] = mainAlternatingColor.get();
                                colors[i + 1] = accentAlternatingColor.get();
                                i++;
                            }
                            setLedStrip(colors);

                            tempAlternatingColor.set(mainAlternatingColor.get());
                            mainAlternatingColor.set(accentAlternatingColor.get());
                            accentAlternatingColor.set(tempAlternatingColor.get());
                        })
                        .andThen(new WaitCommand(0.25)).repeatedly()
                        .withName("ALTERNATING_MOVING, main: " + mainColor.toString() + ", accent: " + accentColor.toString());

                break;

            case RANDOM:
                command = this.runOnce(()-> {
                            Arrays.fill(colors, new Color(rnd.nextInt(255), rnd.nextInt(255), rnd.nextInt(255)));
                            setLedStrip(colors);
                        }).andThen(new WaitCommand(1))
                        .repeatedly().withName("RANDOM");
                break;

            case BLINKING:
                command = Commands.repeatingSequence(
                        new InstantCommand(() -> {
                            Arrays.fill(colors, mainColor);
                            setLedStrip(colors);
                        }, this),
                        new WaitCommand(0.15),
                        new InstantCommand(() -> {
                            Arrays.fill(colors, accentColor);
                            setLedStrip(colors);
                        }, this),
                        new WaitCommand(0.15)
                ).withName("BLINKING, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case TRAIN:
                command = new InstantCommand(() -> {
                    Arrays.fill(colors, mainColor);
                    for (int j = 0; j < trainLength; j++) colors[MathUtil.clamp(j + tailIndex - 1, 0 , LENGTH)] = accentColor;
                    this.tailIndex = invert.get() ? this.tailIndex - 1 : this.tailIndex + 1;
                    if (this.tailIndex == LENGTH - trainLength || this.tailIndex == 0) invert.set(!invert.get());
                    setLedStrip(colors);
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
                    setLedStrip(colors);
                }, this)
                        .andThen(new WaitCommand(0.01)).repeatedly()
                        .withName("TRAIN_CIRCLE, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case RAINBOW:
                AtomicInteger h = new AtomicInteger();
                int s = 255, v = 255;
                command = this.runOnce(()-> {
                    Arrays.fill(colors, Color.fromHSV(h.get(), s, v));
                    if (h.get() <= 180) h.incrementAndGet();
                    else h.set(0);
                }).repeatedly().withName("RAINBOW");

            default:
                break;
        }

        return command.ignoringDisable(true).finallyDo((__) -> restoreLEDs());
    }

    public Command applyPatternCommand(LEDPattern pattern, Color color) {
        return applyPatternCommand(pattern, color, OFF.color);
    }

    public Command controllableLedCommand(DoubleSupplier offset, Color mainColor, Color accentColor){
        Color[] colors = new Color[LENGTH];
        Arrays.fill(colors, mainColor);
        int trainLength = (int) Math.max(LENGTH * 0.15, 1.0);

        return new RunCommand(()->{
            this.offset -= offset.getAsDouble();
            shiftTrain(colors, mainColor, accentColor, trainLength, MathUtil.clamp((int) this.offset, -1, 1));

            if (this.offset >= 1 || this.offset <= -1) this.offset = 0;

            setLedStrip(colors);
        }, this)
                .ignoringDisable(true).finallyDo((__)-> restoreLEDs())
                .withName("CONTROLLABLE, main: " + mainColor + ", accent: " + accentColor);
    }

    public void restoreLEDs() {
        CommandScheduler.getInstance().cancel(
                CommandScheduler.getInstance().requiring(this));
    }

    public enum LEDPattern {
        OFF,
        RAINBOW,
        SOLID,
        ALTERNATING_STATIC,
        ALTERNATING_MOVING,
        TRAIN_CIRCLE,
        TRAIN,
        RANDOM,
        BLINKING;

    }

    public Color getAllianceColor(){
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) return Colors.BLUE.color;
        else if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) return Colors.RED.color;
        else return Colors.WHITE.color;
    }

    private void setLedStrip(Color[] colors) {
        this.currentColors = colors;
        for (int i = 0; i < colors.length; i++) {
            buffer.setLED(i, Color.balance(colors[i]));
        }

        leds.setData(buffer);
    }

    private void shiftTrain(Color[] colors, Color mainColor, Color trainColor, int trainLength, int offset){
        tailIndex = findTailIndex(colors, trainColor);
        Arrays.fill(colors, mainColor);
        for (int i = 0; i < trainLength; i++) {
            colors[stayInBounds(i + tailIndex + offset, colors.length)] = trainColor;
        }
    }

    private int findHeadIndex(Color[] colors, Color trainColor) {
        for (int i = colors.length - 1; i >= 0; i--) {
            if (colors[i].equals(trainColor)) return i;
        }
        return -1;
    }
    private int findTailIndex(Color[] colors, Color trainColor) {
        for (int i = 0; i < colors.length; i++) {
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