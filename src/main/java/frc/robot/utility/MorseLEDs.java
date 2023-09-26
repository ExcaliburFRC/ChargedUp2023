package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDs;

import java.util.HashMap;
import java.util.Map;

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

    public MorseLEDs(){
        for (int i = 0; i < english.length; i++) {
            morseMap.put(english[i], morse[i]);
        }
    }

    public static String textToMorse(String txt){
        txt = txt.toLowerCase();
        StringBuilder builder = new StringBuilder();
        for (Character chr: txt.toCharArray()) {
            builder.append(morseMap.get(chr));
            builder.append(' ');
        }
        return builder.toString();
    }

    public static Command textToLeds(String txt, Color ledColor){
        LEDs leds = LEDs.getInstance();
        SequentialCommandGroup command = new SequentialCommandGroup();
        String morse = textToMorse(txt);
        System.out.println("morse: " + morse);

        for (int i = 0; i < morse.length(); i++) {
            if (morse.charAt(i) == '/'){
                command.addCommands(leds.applyPatternCommand(OFF, ledColor).withTimeout(1)); // delay for space
                continue;
            }

            double timeout = 0;
            if (morse.charAt(i) == '.') timeout = 0.5; // delay for dot
            if (morse.charAt(i) == '-') timeout = 1; // delay for stroke

            command.addCommands(
                    new PrintCommand("lighting with " + timeout + " timeout"),
                    leds.applyPatternCommand(SOLID, ledColor).withTimeout(timeout),
                    leds.applyPatternCommand(OFF, ledColor).withTimeout(0.3) // delay between letters
            );
        }

        return command;
    }
}
