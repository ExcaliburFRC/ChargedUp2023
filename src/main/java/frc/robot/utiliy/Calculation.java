package frc.robot.utiliy;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.Constants.Coordinates.middleAxisXValue;

public class Calculation {
    public static boolean isBlueAlliance(){
        return DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);
    }

    public static Translation2d toOppositeAlliance(Translation2d lastPoint) {
        double dis = Math.abs(middleAxisXValue - lastPoint.getX());
        return new Translation2d(
                lastPoint.getX() < middleAxisXValue ? middleAxisXValue + dis : middleAxisXValue - dis,
                lastPoint.getY());
    }
}
