package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.RollerGripperConstants.*;

public class Rollergripper extends SubsystemBase {
    private final CANSparkMax leader = new CANSparkMax(RIGHT_ROLLER_MOTOR_ID, kBrushless);
    private final CANSparkMax follower = new CANSparkMax(LEFT_ROLLER_MOTOR_ID, kBrushless);

    private final DigitalInput beambreak = new DigitalInput(BEAMBREAK_PORT);
    public final Trigger beambreakTrigger = new Trigger(() -> !beambreak.get());

    public Rollergripper() {
        leader.restoreFactoryDefaults();
        leader.clearFaults();
        leader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leader.setInverted(true);

        follower.restoreFactoryDefaults();
        follower.clearFaults();
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.follow(leader, true);

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
        return Commands.runEnd(
                () -> leader.set(speed),
                ()-> {},
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
                        () -> leader.set(0.5),
                        leader::stopMotor,
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
                        () -> leader.set(-0.025 - offset),
                        leader::stopMotor,
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
                setRollerGripperMotor(0.1).withTimeout(0.25),
                setRollerGripperMotor(0).withTimeout(0.25),
                beambreakTrigger)
                .repeatedly();
    }

    private void initShuffleboardData() {
        Arm.armTab.addBoolean("isConeDetected", beambreakTrigger)
                .withPosition(10, 4).withSize(4, 2);
    }
}
