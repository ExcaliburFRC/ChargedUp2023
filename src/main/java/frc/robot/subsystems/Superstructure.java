package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ArmConstants.Setpoints.*;
import static frc.robot.utility.Colors.GREEN;
import static frc.robot.utility.Colors.RED;

public class Superstructure {
    public final Arm arm = new Arm();
    public final Rollergripper rollergripper = new Rollergripper();

    public Superstructure() {}

    public Command intakeFromShelfCommand() {
        return new SequentialCommandGroup(
                arm.moveToLengthCommand(MIDDLE.setpoint),
                new InstantCommand(() -> Shuffleboard.selectTab("armCamera")),

                new ParallelCommandGroup(
                        rollergripper.intakeCommand(),
                        arm.holdSetpointCommand(SHELF_EXTENDED.setpoint),
                        LEDs.getInstance().applyPatternCommand(LEDs.LEDPattern.BLINKING, RED.color))
                        .until(rollergripper.beambreakTrigger.debounce(0.15)),

                new ParallelCommandGroup(
                        RobotContainer.selectDriveTabCommand(),
                        arm.holdSetpointCommand(SHELF_RETRACTED.setpoint),
                        LEDs.getInstance().applyPatternCommand(LEDs.LEDPattern.SOLID, GREEN.color))
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

    // this command is used when the robot shoots a cube to the high shelf, it moves the Arm, so
    // it doesn't interfere with the shooter
    public Command leanBackCommand() {
        return arm.holdSetpointCommand(CLOSED.setpoint)
                .finallyDo((__) -> arm.lockArmCommand(rollergripper.beambreakTrigger).schedule());
    }

//    public Command placeOnHeightCommand(double height) {
//        return new SelectCommand(
//                Map.of(
//                        LOW_RPM, placeOnLowCommand(),
//                        MID_RPM, placeOnMidSequentially(),
//                        HIGH_RPM, new InstantCommand(() -> {
//                        })), // arm mechanics doesn't allow high cone placement in autonomous
//                () -> height);
//    }
}