package frc.robot.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

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

    public static double deadband(double value){
        return deadband(value, Constants.SwerveConstants.kDeadband);
    }

    public static double deadband(double value, double deadband){
        return MathUtil.applyDeadband(value, deadband);
    }
}
