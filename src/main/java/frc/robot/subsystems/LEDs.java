package frc.robot.subsystems;

import static frc.robot.Constants.LedsConstants.LEDS_LENGTH;
import static frc.robot.Constants.LedsConstants.LEDS_PORT;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;

public class LEDs extends SubsystemBase {
    private final AddressableLED addressableLEDs = new AddressableLED(LEDS_PORT);
    private final AddressableLEDBuffer leds = new AddressableLEDBuffer(LEDS_LENGTH);

    private double brightness = 1;

    public double getBrightness() {
        return brightness;
    }

    public void setBrightness(double brightness) {
        this.brightness = brightness;
    }

    public LEDs() {
        addressableLEDs.setData(leds);
        addressableLEDs.setLength(LEDS_LENGTH);
        addressableLEDs.start();
    }



    void setLedColor(Color[] colors){
        for (int i = 0; i < colors.length; i++) {
            leds.setLED(i, colors[i]);
        }
    }

    Color applyBrightness(double brightness, Color color){
        this.brightness = brightness;
        return new Color(color.red * this.brightness, color.green * brightness, color.blue * brightness);
    }

    Color applyBrightness(Color color){
        return applyBrightness(this.brightness, color);
    }
}
