package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
    private final CANSparkMax angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax angleFollowerMotor = new CANSparkMax(ANGLE_FOLLOWER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax lengthMotor = new CANSparkMax(LENGTH_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final RelativeEncoder lengthEncoder;

    private final DutyCycleEncoder absAngleEncoder = new DutyCycleEncoder(ABS_ANGLE_ENCODER_CHANNEL);

    private final DigitalInput upperLimitSwitch = new DigitalInput(UPPER_LIMIT_SWITCH_ID);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_SWITCH_ID);

    private final Trigger armFullyOpenedTrigger = new Trigger(() -> !upperLimitSwitch.get());
    private final Trigger armFullyClosedTrigger = new Trigger(() -> !lowerLimitSwitch.get());

    private final SparkMaxPIDController angleController;
    private final SparkMaxPIDController lengthController;

    public Arm() {
        angleFollowerMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();
        lengthMotor.restoreFactoryDefaults();

        angleFollowerMotor.follow(angleMotor, false); // TODO: check
        angleMotor.setInverted(false); //TODO: check
        lengthMotor.setInverted(false); //TODO: check

        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        lengthMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleFollowerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        lengthEncoder = lengthMotor.getEncoder();

        lengthEncoder.setPositionConversionFactor(ROT_TO_METER);
        lengthEncoder.setVelocityConversionFactor(RPM_TO_METER_PER_SEC);


        lengthController = lengthMotor.getPIDController();

        lengthController.setP(kP_LENGTH);
        lengthController.setI(0);
        lengthController.setD(0);

        angleController = angleMotor.getPIDController();
        angleController.setP(kP_ANGLE);
        angleController.setI(0);
        angleController.setD(0);

        angleMotor.setClosedLoopRampRate(ARM_RAMP_RATE);
    }

    public Command manualCommand(DoubleSupplier angleJoystick, DoubleSupplier lengthJoystick) {
        return new RunCommand(
                () -> {
                    lengthMotor.set(lengthJoystick.getAsDouble());
                    angleMotor.set(angleJoystick.getAsDouble());
                }, this);
    }

    public Command calibrateLengthEncoderCommand() {
        return new RunCommand(
                () -> lengthMotor.set(-0.1))
                .until(armFullyClosedTrigger)
                .andThen(new InstantCommand(
                        () -> {
                            lengthMotor.stopMotor();
                            lengthEncoder.setPosition(MINIMAL_LENGTH_METERS);
                        }
                ));
    }

    private double getArmDegrees() {
        double wantedAngle = absAngleEncoder.getAbsolutePosition() - ABS_ENCODER_OFFSET_ANGLE_DEG;
        if (wantedAngle < 0) wantedAngle += 1;
        return wantedAngle * 360;
    }

    private double getProfiledAngle() {//TODO: change from 0 to 360 to -180 to 180
        double realAngle = getArmDegrees();
        if (realAngle > (PHYSICAL_BACK_MAX_ARM_ANGLE_DEG + PHYSICAL_FRONT_MAX_ARM_ANGLE_DEG) / 2)
            return 360 - realAngle;
        return realAngle;
    }

    public Command holdSetpointCommand(Translation2d setpoint) {
        return moveToLengthCommand(setpoint).alongWith(moveToAngleCommand(setpoint));
    }

    private Command moveToLengthCommand(Translation2d setPoint) {
        return new ProxyCommand(
                () -> new TrapezoidProfileCommand(
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(kMaxLinearVelocity, kMaxAngularAcceleration),
                        new TrapezoidProfile.State(setPoint.getNorm(), 0),
                        new TrapezoidProfile.State(lengthEncoder.getPosition(), lengthEncoder.getVelocity())),
                state -> {
                    double feedforward = kS_LENGTH * Math.signum(state.velocity)
                            + kG_LENGTH * Math.sin(Units.degreesToRadians(getArmDegrees()))
                            + kV_LENGTH * state.velocity;

                    lengthController.setReference(state.position, CANSparkMax.ControlType.kPosition,
                            0,
                            feedforward, SparkMaxPIDController.ArbFFUnits.kVoltage);
                }, this));
    }

    public Command moveToAngleCommand(Translation2d setPoint) {
        return new ProxyCommand(
                () -> new TrapezoidProfileCommand(
                        new TrapezoidProfile(
                                new TrapezoidProfile.Constraints(kMaxAngularVelocity, kMaxAngularAcceleration),
                                new TrapezoidProfile.State(setPoint.getAngle().getDegrees(), 0),
                                new TrapezoidProfile.State(getProfiledAngle(), 0)
                        ),
                        state -> {
                            double feedforward =
                                    kS_ANGLE * Math.signum(state.velocity)
                                            + kG_ANGLE.get(lengthEncoder.getPosition()) * Math.cos(state.position)
                                            + kV_ANGLE * state.velocity;
                            angleController.setReference(state.position, CANSparkMax.ControlType.kPosition,
                                    0,
                                    feedforward, SparkMaxPIDController.ArbFFUnits.kVoltage);
                        }
                ));
    }

    private double getAbsEncoderPos() {
        return absAngleEncoder.getAbsolutePosition() - ABS_ENCODER_OFFSET_ANGLE_DEG < 0 ?
                360 - (absAngleEncoder.getAbsolutePosition() + ABS_ENCODER_OFFSET_ANGLE_DEG) :
                absAngleEncoder.getAbsolutePosition() - ABS_ENCODER_OFFSET_ANGLE_DEG;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("fullyOpened", armFullyOpenedTrigger, null);
        builder.addBooleanProperty("fullyClosed", armFullyClosedTrigger, null);
    }
}
