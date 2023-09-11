package frc.robot.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class Calculation {
    public static boolean isBlueAlliance(){
        return DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);
    }

    public static double deadband(double value){
        return deadband(value, Constants.SwerveConstants.kDeadband);
    }

    public static double deadband(double value, double deadband){
        return MathUtil.applyDeadband(value, deadband);
    }
}
