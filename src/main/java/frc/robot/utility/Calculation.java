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

    // converts value from a -1 to 1 range to a 0 to 1 range
    public static double convertJoystickRange(double val){
        return (val + 1) / 2;
    }

    public static double getSwerveDeceleratorVal(double triggerVal){
        return Math.max(1.0 - convertJoystickRange(triggerVal), 0.1);
    }
}
