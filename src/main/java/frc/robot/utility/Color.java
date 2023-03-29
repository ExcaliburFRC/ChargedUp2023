package frc.robot.utility;

public class Color extends edu.wpi.first.wpilibj.util.Color {
    public Color(double red, double green, double blue) {
        super(green, red, blue);
    }

    public Color(int red, int green, int blue) {
        super(green, red, blue);
    }

    public Color(){
        super(0, 0, 0);
    }

    public static Color balance(Color color) {
        double v = Math.max(Math.max(color.red, color.green), color.blue);
        Color newColor = new Color(color.red, color.green / 2, color.blue / 4);
        double newV = Math.max(Math.max(newColor.red, newColor.green), newColor.blue);
        double ratio = v / newV;
        return new Color(newColor.red * ratio, newColor.green * ratio, newColor.blue * ratio);
    }
}
