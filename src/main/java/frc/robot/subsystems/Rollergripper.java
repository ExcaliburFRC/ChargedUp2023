package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.RollerGripperConstants.*;

public class Rollergripper extends SubsystemBase {
    private final CANSparkMax right = new CANSparkMax(RIGHT_ROLLER_MOTOR_ID, kBrushless);
    private final CANSparkMax left = new CANSparkMax(LEFT_ROLLER_MOTOR_ID, kBrushed);

    private final DigitalInput beambreak = new DigitalInput(BEAMBREAK_PORT);
    public final Trigger beambreakTrigger = new Trigger(() -> !beambreak.get());

    public Rollergripper() {
        right.restoreFactoryDefaults();
        right.clearFaults();
        right.setIdleMode(CANSparkMax.IdleMode.kBrake);
        right.setInverted(true);

        left.restoreFactoryDefaults();
        left.clearFaults();
        left.setIdleMode(CANSparkMax.IdleMode.kBrake);

        initShuffleboardData();
        setDefaultCommand(holdConeCommand());
    }

    /**
     * sets the roller gripper motors speeds
     *
     * @param speed the speed the motors spin at
     * @return the command
     */
    private Command setRollerGripperMotor(double speed) {
        return Commands.runEnd(() ->
                        left.set(speed),
                left::stopMotor,
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
                            right.set(0.5);
                            left.set(0.5);
                        },
//                            Shuffleboard.selectTab("armCamera");,
                        () -> {
                            right.stopMotor();
                            left.stopMotor();
                        },
//                            Shuffleboard.selectTab("driveTab");,
                        this)
                .until(beambreakTrigger.debounce(0.1));
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
                        () -> {
                            right.set(-0.025 - offset);
                            left.set(-0.025 - offset);
                        },
                        () -> {
                            right.stopMotor();
                            left.stopMotor();
                        },
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
    public Command holdConeCommand() {
        return new ConditionalCommand(
                this.runOnce(() -> {
                    right.set(0.05);
                    left.set(0.15);
                }),
                this.runOnce(() -> {
                    right.stopMotor();
                    left.stopMotor();
                    }),
                beambreakTrigger).repeatedly();
    }

    private void initShuffleboardData() {
        Arm.armTab.addBoolean("isConeDetected", beambreakTrigger)
                .withPosition(10, 4).withSize(4, 2);
    }
}
