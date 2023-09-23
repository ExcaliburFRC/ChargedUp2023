package frc.robot.utility;

public enum Colors {
    OFF(new Color(0, 0, 0)),

    TEAM_GOLD(new Color(255, 215, 0)),
    TEAM_BLUE(new Color(1, 34, 101)),

    RED(new Color(255, 0, 0)),
    GREEN(new Color(0, 255, 0)),
    BLUE(new Color(0, 0, 255)),

    WHITE(new Color(255, 255, 255)),

    YELLOW(new Color(255, 255, 0)),
    CYAN(new Color(0, 255, 255)),
    PINK(new Color(255, 0, 255)),

    ORANGE(new Color(255, 165, 0)),
    PURPLE(new Color(160, 32, 240));

    // TODO: add more colors

    public final Color color;

    Colors(Color color) {
        this.color = color;
    }
}