package frc.robot.utiliy;

import edu.wpi.first.wpilibj.DriverStation;

public class Calculation {
    public static boolean isBlueAlliance(){
        return DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);
    }
}
