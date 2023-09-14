package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utility.Limelight;

import java.util.function.BooleanSupplier;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.RollerGripperConstants.*;

public class Rollergripper extends SubsystemBase {
    private final CANSparkMax rightRoller = new CANSparkMax(RIGHT_ROLLER_MOTOR_ID, kBrushless);

    // This is very embarrassing and not at all ideal, please don't laugh at us,
    // we don't have enough budget for a new SparkMax, and we had those lying around.
    private final Spark leftRoller = new Spark(LEFT_ROLLER_MOTOR_PORT);

    private final DigitalInput beambreak = new DigitalInput(BEAMBREAK_PORT);

    public final Trigger beambreakTrigger = new Trigger(() -> !beambreak.get()).whileTrue(holdConeCommand());

    public Rollergripper() {
        rightRoller.restoreFactoryDefaults();
        rightRoller.clearFaults();
        rightRoller.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftRoller.setInverted(true);
        rightRoller.setInverted(true);

        Arm.armTab.addBoolean("isConeDetected", beambreakTrigger)
                .withPosition(10, 4).withSize(4, 2);
        Limelight.armCameraTab.addBoolean("isConeDetected", beambreakTrigger)
                .withSize(2, 8);
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
                            rightRoller.set(0.15);
                            leftRoller.set(0.25);
                            Shuffleboard.selectTab("armCamera");
                        },
                        () -> {
                            rightRoller.stopMotor();
                            leftRoller.stopMotor();
                            Shuffleboard.selectTab("driveTab");
                        },
                        this)
                .until(beambreakTrigger);
    }

    /**
     * ejectCommand
     * <p><b> noInit </b>starts spinning the motors outwards<br>
     * <b>noEnd </b>stops the motors</p>
     * ends when the button senses that a cone is no longer in the roller gripper
     *
     * @return the command
     */
    public Command ejectCommand(double offset) {
        return Commands.runEnd(
                        () -> {
                            rightRoller.set(-0.025 - offset);
                            leftRoller.set(-0.025 - offset);
                        },
                        () -> {
                            rightRoller.stopMotor();
                            leftRoller.stopMotor();
                        },
                        this)
                .until(beambreakTrigger.negate().debounce(0.2));
    }

    public Command ejectCommand() {
        return ejectCommand(0);
    }

    /**
     * waits until the driver wants to release the cone and then returns the eject command
     *
     * @param release the button that needs to be pressed in order to eject
     * @return the eject command after the button was pressed
     */
    public Command releaseCommand(BooleanSupplier release) {
        return new RunCommand(() -> {
        }).until(release).andThen(ejectCommand());
    }

    /**
     * manual command the allows full manual control of the system
     *
     * @param intake  whether the system should currently intake
     * @param outtake whether the system should currently eject
     * @return the command
     */
    public Command manualCommand(Trigger intake, Trigger outtake) {
        return Commands.runEnd(
                () -> {
                    if (intake.getAsBoolean() || outtake.getAsBoolean()) {
                        if (intake.getAsBoolean()) {
                            rightRoller.set(0.6);
                            leftRoller.set(0.6);
                        }
                        if (outtake.getAsBoolean()) {
                            rightRoller.set(-0.1);
                            leftRoller.set(-0.1);
                        }
                    } else {
                        rightRoller.set(0);
                        leftRoller.set(0);
                    }
                },
                () -> {
                    rightRoller.set(0);
                    leftRoller.set(0);
                },
                this
        );
    }

    /**
     * applies a small force to the roller gripper in order to hold it in place
     *
     * @return the command
     */
    private Command holdConeCommand(){
        return new StartEndCommand(
                ()-> rightRoller.set(0.05),
                rightRoller::stopMotor,
                this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("rollerGripper button", beambreakTrigger.getAsBoolean());
    }
}
