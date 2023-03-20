package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;

import java.util.Map;
import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ArmConstants.Setpoints.*;
import static frc.robot.Constants.IntakeConstants.*;

public class Superstructure extends SubsystemBase {
    public final Arm arm = new Arm();
    public final Rollergripper rollergripper = new Rollergripper();

    public Superstructure() {
        rollergripper.setDefaultCommand(rollergripper.holdConeCommand());

        arm.setDefaultCommand(arm.fadeArmCommand().alongWith(arm.stopTelescopeMotors()));
    }

    public Command intakeFromShelfCommand() {
        return arm.moveToLengthCommand(MIDDLE.setpoint).andThen(
                new InstantCommand(() -> Shuffleboard.selectTab("armCamera")),
                rollergripper.intakeCommand().alongWith(arm.holdSetpointCommand(SHELF_EXTENDED.setpoint))
                        .until(rollergripper.beambreakTrigger.debounce(0.3)),
                RobotContainer.selectDriveTabCommand(),
                arm.holdSetpointCommand(SHELF_RETRACTED.setpoint).withTimeout(0.5));
    }

    public Command placeOnHighCommand(BooleanSupplier release) {
        return new SequentialCommandGroup(
                arm.holdSetpointCommand(HIGH_CHECKPOINT.setpoint).withTimeout(1.5),
                arm.holdSetpointCommand(HIGH.setpoint).until(release),
                arm.fadeArmCommand().alongWith(rollergripper.ejectCommand(0.035))
                        .until(rollergripper.beambreakTrigger.negate().debounce(0.1)),
                arm.resetLengthCommand());
    }

    public Command placeOnMidCommand(BooleanSupplier release) {
        return arm.holdSetpointCommand(MID.setpoint).until(release)
                .andThen(arm.setAngleSpeed(-6.5).alongWith(rollergripper.ejectCommand()))
                .until(rollergripper.beambreakTrigger.negate().debounce(0.2));
    }

    public Command placeOnLowCommand() {
        return arm.holdSetpointCommand(LOW.setpoint).withTimeout(0.65)
                .andThen(rollergripper.ejectCommand(0.2))
                .until(rollergripper.beambreakTrigger.negate().debounce(0.05));
    }

    public Command lockArmCommand() {
        return arm.lockArmCommand(rollergripper.beambreakTrigger);
    }

    public Command placeOnHeightCommand(double height) {
        return new SelectCommand(
                Map.of(
                        LOW_RPM, placeOnLowCommand(),
                        MID_RPM, placeOnMidCommand(() -> new WaitCommand(5).isFinished()),
                        HIGH_RPM, placeOnHighCommand(() -> new WaitCommand(7).isFinished())),
                () -> height
        );
    }
}
