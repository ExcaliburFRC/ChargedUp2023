package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Cuber extends SubsystemBase {
    private final CANSparkMax angleMotor = new CANSparkMax
      (Constants.CuberConstants.ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rollersMotor = new CANSparkMax
      (Constants.CuberConstants.ROLLERS_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final Servo servo = new Servo(Constants.CuberConstants.SERVO_CHANNEL);

    public Cuber() {
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rollersMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public Command setServo(double angle) {
        return new InstantCommand(() ->
          servo.setAngle(angle));
    }

    public double getServo() {
        return servo.getAngle();
    }

    public Command resetServo() {
        return new InstantCommand(() ->
          servo.setAngle(0));
    }

    public Command pushCube() {
        return new InstantCommand(() ->
          servo.setAngle(45));
    }

    public Command setRollersSpeed(double speed) {
        return new RunCommand(()-> rollersMotor.set(speed));
    }
    public Command setAngleSpeed(double speed){
        return new RunCommand(()->
          angleMotor.set(speed));
    }

}
