package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDs;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants.LedsConstants.LENGTH;
import static frc.robot.subsystems.LEDs.LEDPattern.OFF;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;

public class MorseLEDs {
    private final char[] english = {
            'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l',
            'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x',
            'y', 'z', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0',
            ',', '.', '?', ' '};

    private final String[] morse = {
            ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..",
            ".---", "-.-", ".-..", "--", "-.", "---", ".---.", "--.-", ".-.",
            "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--..", ".----",
            "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----.",
            "-----", "--..--", ".-.-.-", "..--..", "/"};

    private static final Map<Character, String> morseMap = new HashMap<>();
    private static final MorseLEDs INSTANCE = new MorseLEDs();
    private static final SequentialCommandGroup command = new SequentialCommandGroup();


    public MorseLEDs() {
        for (int i = 0; i < english.length; i++) {
            morseMap.put(english[i], morse[i]);
        }
    }

    private static String textToMorse(String txt) {
        txt = txt.toLowerCase();
        StringBuilder builder = new StringBuilder();
        for (Character chr : txt.toCharArray()) {
            builder.append(morseMap.get(chr));
            builder.append(' ');
        }
        return builder.toString();
    }

    // write a function that translates morse code into led signaling
    // use the LEDs class to set the LEDs to the correct pattern


    public static Command textToLeds(String txt, Color ledColor) {
//        GenericEntry shuffleboardText = Shuffleboard.getTab("driveTab").add("morse text", "").getEntry();

        LEDs leds = LEDs.getInstance();
//            String morse = textToMorse(shuffleboardText.getString("").equals("") ? txt : shuffleboardText.getString(""));
        String morse = textToMorse(txt);

        for (int i = 0; i < morse.length(); i++) {
//                int finalI = i;
//                command.addCommands(new InstantCommand(()->
//                DriverStation.reportError(String.valueOf(morse.charAt(finalI)), false)
//                ));

            if (morse.charAt(i) == ' ') {
                command.addCommands(leds.applyPatternCommand(OFF, ledColor).withTimeout(0.8)); // delay between letters
                continue;
            }
            if (morse.charAt(i) == '/') {
                command.addCommands(leds.applyPatternCommand(OFF, ledColor).withTimeout(1.5)); // delay between words
                continue;
            }

            double timeout = 0;
            if (morse.charAt(i) == '.') timeout = 0.5; // delay for dot
            if (morse.charAt(i) == '-') timeout = 1; // delay for stroke

            command.addCommands(
                    leds.applyPatternCommand(SOLID, ledColor).withTimeout(timeout),
                    leds.applyPatternCommand(OFF, ledColor).withTimeout(0.3) // delay between signs
            );
        }

        return command;
    }

    public static Command textToAddressableLeds(String txt, Color color){
        LEDs leds = LEDs.getInstance();
        Color[] colors = new Color[LENGTH];
        Arrays.fill(colors, Colors.OFF.color);
        int j = 0;

        String morse = textToMorse(txt);
        String revMorse = new StringBuilder(morse).reverse().toString();

        if (getLEDsLength(morse) >= colors.length) return new PrintCommand("morse code too long for leds!");

        for (int i = 0; i < revMorse.length(); i ++) {
            if (revMorse.charAt(i) == ' ') {
                colors[j] = Colors.OFF.color;
            }

            if (revMorse.charAt(i) == '.') {
                    colors[j] = color;
                    colors[j + 1] = Colors.OFF.color;
                    j++;
            }

            if (revMorse.charAt(i) == '-') {
                colors[j] = color;
                colors[j + 1] = color;
                colors[j + 2] = Colors.OFF.color;
                j += 2;
            }

            if (revMorse.charAt(i) == '/') {
                colors[j] = Colors.OFF.color;
            }

            j ++;
        }

//        return leds.setLEDsCommand(colors);
        return new InstantCommand(()->{});
    }

    private static int getLEDsLength(String morse){
        int counter = 0;

        for (int i = 0; i < morse.length(); i++) {
            if (morse.charAt(i) == ' ') counter ++;
            if (morse.charAt(i) == '.') counter += 2;
            if (morse.charAt(i) == '-') counter += 3;
            if (morse.charAt(i) == '/') counter += 1;
        }

        return counter;
    }
}