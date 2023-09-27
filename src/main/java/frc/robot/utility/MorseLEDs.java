package frc.robot.utility;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    private static final SequentialCommandGroup command = new SequentialCommandGroup();


    public MorseLEDs(){
        for (int i = 0; i < english.length; i++) {
            morseMap.put(english[i], morse[i]);
        }
    }

    private static String textToMorse(String txt){
        txt = txt.toLowerCase();
        StringBuilder builder = new StringBuilder();
        for (Character chr: txt.toCharArray()) {
            builder.append(morseMap.get(chr));
            builder.append(' ');
        }
        return builder.toString();
    }

    // write a function that translates morse code into led signaling
    // use the LEDs class to set the LEDs to the correct pattern


    public static Command textToLeds(String txt, Color ledColor){
        GenericEntry shuffleboardText = Shuffleboard.getTab("driveTab").add("morse text", "").getEntry();

            LEDs leds = LEDs.getInstance();
            String morse = textToMorse(shuffleboardText.getString("").equals("") ? txt : shuffleboardText.getString(""));
            System.out.println("morse: " + morse);


            for (int i = 0; i < morse.toCharArray().length; i++) {
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

//    public static Command textToLeds(String txt, Color ledColor){
//        AtomicInteger index = new AtomicInteger(0);
//        String morse = textToMorse(txt);
//
//        return new InstantCommand(()-> index.set(0)).andThen(
//                Commands.repeatingSequence(
//                        new InstantCommand(()-> DriverStation.reportError(String.valueOf(morse.charAt(index.get())), false)),
//                        getLEDsCommand(morse.charAt(index.get()), ledColor),
//                        LEDs.getInstance().applyPatternCommand(OFF, ledColor).withTimeout(0.15),
//                        new InstantCommand(index::incrementAndGet)))
//                .until(()-> index.get() == morse.length()).ignoringDisable(true);
//    }
//
//    private static Command getLEDsCommand(char chr, Color ledColor){
//        LEDs leds = LEDs.getInstance();
//
//        return new ConditionalCommand(
//                leds.applyPatternCommand(OFF, ledColor).withTimeout(4), // delay for / (between words)
//                new PrintCommand("A"),
//                new ConditionalCommand(
//                        leds.applyPatternCommand(SOLID, ledColor).withTimeout(1), // delay for .
//                        new PrintCommand("B"),
//                        new ConditionalCommand(
//                                leds.applyPatternCommand(SOLID, ledColor).withTimeout(2), // delay for -
//                                new PrintCommand("C"),
//                                leds.applyPatternCommand(OFF, ledColor).withTimeout(0.5), // delay for space (between letters)
//                                new PrintCommand("D"),
//                                ()-> chr == '-'),
//                        ()-> chr == '.'),
//                ()-> chr == '/'
//        );
//    }
}
