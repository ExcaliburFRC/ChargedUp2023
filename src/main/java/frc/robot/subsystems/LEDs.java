package frc.robot.subsystems;

import static frc.robot.Constants.LedsConstants.LEDS_LENGTH;
import static frc.robot.Constants.LedsConstants.LEDS_PORT;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.utility.Color;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds = new AddressableLED(LEDS_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDS_LENGTH);

    private double brightness = 1;

    public double getBrightness() {
        return brightness;
    }

    public void setBrightness(double brightness) {
        this.brightness = brightness;
    }

    public LEDs() {
        leds.setData(buffer);
        leds.setLength(LEDS_LENGTH);
        leds.start();
    }

    private void setLedColor(Color[] colors){
        for (int i = 0; i < colors.length; i++) {
            buffer.setLED(i, Color.balance(colors[i]));
        }
        leds.setData(buffer);
    }

    Color applyBrightness(double brightness, Color color){
        this.brightness = brightness;
        return new Color(color.red * this.brightness, color.green * this.brightness, color.blue * this.brightness);
    }

    Color applyBrightness(Color color){
        return applyBrightness(this.brightness, color);
    }
}
