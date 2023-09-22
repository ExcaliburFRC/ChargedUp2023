package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utility.Limelight;

import java.util.function.BooleanSupplier;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.RollerGripperConstants.*;

public class Rollergripper extends SubsystemBase {
    private final CANSparkMax rollers = new CANSparkMax(LEFT_ROLLER_MOTOR_ID, kBrushless);
    private final CANSparkMax follower = new CANSparkMax(RIGHT_ROLLER_MOTOR_ID, kBrushless);

    private final DigitalInput beambreak = new DigitalInput(BEAMBREAK_PORT);
    public final Trigger beambreakTrigger = new Trigger(() -> !beambreak.get());

    public Rollergripper() {
        follower.restoreFactoryDefaults();
        follower.clearFaults();
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.follow(rollers, true);

        rollers.restoreFactoryDefaults();
        rollers.clearFaults();
        rollers.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rollers.setInverted(false);

        Arm.armTab.addBoolean("isConeDetected", beambreakTrigger)
                .withPosition(10, 4).withSize(4, 2);
        Limelight.armCameraTab.addBoolean("isConeDetected", beambreakTrigger)
                .withSize(2, 8);

        setDefaultCommand(setRollerGripperMotor(0.2));
    }

    /**
     * sets the roller gripper motors speeds
     *
     * @param speed the speed the motors spin at
     * @return the command
     */
    private Command setRollerGripperMotor(double speed) {
        return Commands.runEnd(() -> {
                    rollers.set(speed);
                    follower.set(speed);
                },
                () -> {
                    rollers.stopMotor();
                    follower.stopMotor();
                },
                this);
    }

    /**
     * intakeCommand
     * <p><b> noInit </b>starts the motors <br>
     * <b>noEnd </b>stops the motors</p>
     * <b>ends when the button senses that a cone was collected into the roller gripper </b>
     *
     * @return the command
     */
    public Command intakeCommand() {
        return Commands.runEnd(
                        () -> {
                            rollers.set(0.75);
                            Shuffleboard.selectTab("armCamera");
                        },
                        () -> {
                            rollers.stopMotor();
                            Shuffleboard.selectTab("driveTab");
                        },
                        this)
                .until(beambreakTrigger);
    }

    /**
     * ejectCommand
     * <p><b> noInit </b>starts spinning the motors outwards<br>
     * <b>noEnd </b>stops the motors</p>
     * ends when the button senses that a cone is not longer in the roller gripper
     *
     * @return the command
     */
    public Command ejectCommand(double offset) {
        return Commands.runEnd(
                        () -> rollers.set(-0.025 - offset),
                        rollers::stopMotor,
                        this) //runEnd ends here
                .until(beambreakTrigger.negate().debounce(0.2));
    }

    public Command ejectCommand() {
        return ejectCommand(0);
    }

    /**
     * applies a small force to the roller gripper in order to hold it in place
     *
     * @return the command
     */
    private Command holdConeCommand() {
        return new ConditionalCommand(
                Commands.runEnd(() -> rollers.set(0.25), rollers::stopMotor, this).withTimeout(0.05),
                new InstantCommand(() -> {
                }, this),
                beambreakTrigger
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("rollerGripper button", beambreakTrigger.getAsBoolean());
    }
}
