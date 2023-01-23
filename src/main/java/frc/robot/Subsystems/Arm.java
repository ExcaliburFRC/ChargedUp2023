package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
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

    private final RelativeEncoder lengthEncoder;

    private final DutyCycleEncoder absAngleEncoder = new DutyCycleEncoder(ABS_ANGLE_ENCODER_CHANNEL);

    private final DigitalInput upperLimitSwitch = new DigitalInput(UPPER_LIMIT_SWITCH_ID);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_SWITCH_ID);

    private final Trigger armFullyOpenedTrigger = new Trigger(() -> !upperLimitSwitch.get());
    private final Trigger armFullyClosedTrigger = new Trigger(() -> !lowerLimitSwitch.get());

    private final PIDController angleController;
    private final PIDController lengthController;


    public Arm() {
        angleMotor.restoreFactoryDefaults();
        lengthMotor.restoreFactoryDefaults();

        angleMotor.setInverted(false); //TODO: check
        lengthMotor.setInverted(false); //TODO: check

        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        lengthMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        lengthEncoder = lengthMotor.getEncoder();

        lengthEncoder.setPositionConversionFactor(ROT_TO_METER);
        lengthEncoder.setVelocityConversionFactor(RPM_TO_METER_PER_SEC);

        lengthController = new PIDController(kP_LENGTH, kI_LENGTH, kD_LENGTH);
        angleController = new PIDController(kP_ANGLE, kI_ANGLE, kD_ANGLE);

        lengthController.setTolerance(LENGTH_TOLERANCE);
        angleController.setTolerance(ANGLE_TOLERANCE);

        angleMotor.setOpenLoopRampRate(ARM_RAMP_RATE);
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

    public Command setTranslationCommand(Translation2d setPoint) {
        double lengthSetPoint = setPoint.getNorm();
        double angleSetPoint = setPoint.getAngle().getDegrees();
        return new RunCommand(
                () -> {
                    if (achievableTranslation(setPoint)) {
                        angleMotor.set(angleController.calculate(
                                getAbsEncoderPos(),
                                angleSetPoint));
                        lengthMotor.set(lengthController.calculate(
                                getLengthMeter(),
                                lengthSetPoint));
                    }
                });
    }

    private boolean achievableTranslation(Translation2d target) {
        return target.getNorm() >= MINIMAL_LENGTH_METERS && target.getNorm() <= MINIMAL_LENGTH_METERS * 2 &&
                (target.getAngle().getDegrees() <= PHYSICAL_BACK_MAX_ARM_ANGLE_DEG ||
                  target.getAngle().getDegrees() >= PHYSICAL_FRONT_MAX_ARM_ANGLE_DEG);
    } // TODO: find the max length multiplier

    private double getAbsEncoderPos() {
        return absAngleEncoder.getAbsolutePosition() - ABS_ENCODER_OFFSET_ANGLE_DEG < 0 ?
                360 - (absAngleEncoder.getAbsolutePosition() + ABS_ENCODER_OFFSET_ANGLE_DEG) :
                absAngleEncoder.getAbsolutePosition() - ABS_ENCODER_OFFSET_ANGLE_DEG;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("fullyOpened", armFullyOpenedTrigger, null);
        builder.addBooleanProperty("fullyClosed", armFullyClosedTrigger, null);
    }
}
