package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;

import java.util.Map;
import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ArmConstants.Setpoints.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.LedsConstants.Colors.*;

public class Superstructure extends SubsystemBase {
    public final Arm arm = new Arm();
    public final Rollergripper rollergripper = new Rollergripper();

    public Superstructure() {
        rollergripper.setDefaultCommand(rollergripper.holdConeCommand());

        arm.setDefaultCommand(arm.fadeArmCommand().alongWith(arm.stopTelescopeMotors()));
    }

    public Command intakeFromShelfCommand() {
        return new SequentialCommandGroup(
                arm.moveToLengthCommand(MIDDLE.setpoint),
                new InstantCommand(() -> Shuffleboard.selectTab("armCamera")),
                new ParallelCommandGroup(
                        rollergripper.intakeCommand(),
                        arm.holdSetpointCommand(SHELF_EXTENDED.setpoint),
                        LEDs.getInstance().applyPatternCommand(LEDs.LEDPattern.BLINKING, RED.color, OFF.color))
                        .until(rollergripper.beambreakTrigger.debounce(0.15)),
                RobotContainer.selectDriveTabCommand(),
                arm.holdSetpointCommand(SHELF_RETRACTED.setpoint)
                        .alongWith(LEDs.getInstance().applyPatternCommand(LEDs.LEDPattern.SOLID, GREEN.color, OFF.color))
                        .withTimeout(0.5));
    }

    public Command placeOnHighCommand(BooleanSupplier release) {
        return new SequentialCommandGroup(
                arm.holdSetpointCommand(HIGH_CHECKPOINT.setpoint).withTimeout(1.25),
                arm.holdSetpointCommand(HIGH.setpoint).until(release),
                arm.fadeArmCommand().alongWith(rollergripper.ejectCommand(0.03))
                        .until(rollergripper.beambreakTrigger.negate().debounce(0.1)),
                arm.resetLengthCommand());
    }

    public Command placeOnMidCommand(BooleanSupplier release) {
        return arm.holdSetpointCommand(MID.setpoint).until(release)
                .andThen(arm.setAngleSpeed(-6.5).alongWith(rollergripper.ejectCommand()))
                .until(rollergripper.beambreakTrigger.negate().debounce(0.1));
    }

    public Command placeOnMidSequentially() {
        return arm.moveToAngleCommand(MID.setpoint)
                .alongWith(new WaitCommand(1).andThen(arm.moveToLengthCommand(MID.setpoint)))
                .until(() -> arm.armAtSetpoint(MID.setpoint)).andThen(placeOnMidCommand(() -> true));
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
                        MID_RPM, placeOnMidSequentially(),
                        HIGH_RPM, new InstantCommand(() -> {
                        })), // arm mechanics doesn't allow high cone placement in autonomous
                () -> height);
    }
}