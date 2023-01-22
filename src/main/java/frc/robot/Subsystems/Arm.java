package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
    private final CANSparkMax angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax lengthMotor = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder angleEncoder;
    private final RelativeEncoder lengthEncoder;

    private final DutyCycleEncoder absAngleEncoder = new DutyCycleEncoder(ABS_ANGLE_ENCODER_CHANNEL);;

    private final DigitalInput upperLimitSwitch = new DigitalInput(UPPER_LIMIT_SWITCH_ID);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_SWITCH_ID);

    private final Trigger armFullyOpenedTrigger = new Trigger(()-> !upperLimitSwitch.get());
    private final Trigger armFullyClosedTrigger = new Trigger(()-> !lowerLimitSwitch.get());

    public Arm() {
        angleMotor.setInverted(false); //TODO: check
        lengthMotor.setInverted(false); //TODO: check

        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        lengthMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        angleEncoder = angleMotor.getEncoder();
        lengthEncoder = lengthMotor.getEncoder();

        angleEncoder.setPositionConversionFactor(360); // degrees
        angleEncoder.setVelocityConversionFactor(RPM_TO_ROT_PER_SEC); // rpm/rot's

        lengthEncoder.setPositionConversionFactor(ROT_TO_METER);
        lengthEncoder.setVelocityConversionFactor(RPM_TO_METER_PER_SEC);


        resetAngle();
    }

    public Command manualCommand(DoubleSupplier angleJoystick, DoubleSupplier lengthJoystick) {
        return new RunCommand(
                () -> {
                    lengthMotor.set(lengthJoystick.getAsDouble());
                    angleMotor.set(angleJoystick.getAsDouble());
                }, this);
    }

    public double getLengthMeter() {
        return MINIMAL_LENGTH_METERS + lengthEncoder.getPosition();
    }

    public void resetAngle() {
        angleEncoder.setPosition(0);
    }
}
