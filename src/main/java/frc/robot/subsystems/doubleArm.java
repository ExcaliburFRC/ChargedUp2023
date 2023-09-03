package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;

public class doubleArm extends SubsystemBase {
    private CANSparkMax joint1 = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax joint2 = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    double length1 = 0, length2 = 0;
    private double getAngle(CANSparkMax motor){
        return joint1.getEncoder().getPosition();
    }
    private Translation2d getPoint(){
        Translation2d point1 = new Translation2d(length1, Rotation2d.fromDegrees(getAngle(joint1)));
        Translation2d point2 = new Translation2d(length2, Rotation2d.fromDegrees(getAngle(joint1) + getAngle(joint2)%360));
        return point1.plus(point2);
    }

    public Command moveToAngleCommand(Translation2d setpoint, CANSparkMax motor) {
        return Commands.runEnd(() -> {
            double setpointDeg = setpoint.getAngle().getDegrees() < 0? setpoint.getAngle().getDegrees() + 360 : setpoint.getAngle().getDegrees();
            double pid = kP_ANGLE * (setpointDeg - getAngle(motor));
            double feedforward = kS_ANGLE * Math.signum(pid) + kG_ANGLE * setpoint.getAngle().getCos();

            if (getAngle(motor) <= 80 || getAngle(motor) >= 220) {
                DriverStation.reportError("odd arm angle reading while motor running! please check.", false);
                motor.stopMotor();
            } else motor.setVoltage(pid + feedforward);

        }, motor::stopMotor, this);
    }
    public Command goToPoint(Translation2d point){

    }
}
