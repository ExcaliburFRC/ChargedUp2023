package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Map;
import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ArmConstants.Setpoints.*;
import static frc.robot.utility.Colors.ORANGE;

public class Superstructure {
    public final Arm arm = new Arm();
    public final Rollergripper rollergripper = new Rollergripper();

    public Command intakeFromShelfCommand() {
        return new SequentialCommandGroup(
                arm.moveToLengthCommand(INTAKE_CHECKPOINT.setpoint),
                new ParallelRaceGroup(
                        rollergripper.intakeCommand(),
                        arm.holdSetpointCommand(SHELF_EXTENDED.setpoint),
                        LEDs.getInstance().applyPatternCommand(LEDs.LEDPattern.BLINKING, ORANGE.color)),
                rollergripper.holdConeCommand().alongWith(
                arm.holdSetpointCommand(SHELF_RETRACTED.setpoint))
        );
    }

    public Command placeOnHighCommand(Trigger release) {
        return new SequentialCommandGroup(
                arm.holdSetpointCommand(BUMPER.setpoint).withTimeout(0.3),
                arm.holdSetpointCommand(HIGH_CHECKPOINT.setpoint).withTimeout(1.25),
                arm.holdSetpointCommand(HIGH.setpoint).until(release),
//                arm.osscilateArmCommand(5).until(release.negate()),
                rollergripper.ejectCommand().alongWith(arm.setAngleSpeed(0)).until(rollergripper.beambreakTrigger.negate()),
                arm.holdSetpointCommand(SHELF_RETRACTED.setpoint));
    }

    public Command placeOnMidCommand(BooleanSupplier release) {
        return new SequentialCommandGroup(
                arm.holdSetpointCommand(MID.setpoint).until(release),
                rollergripper.ejectCommand(0).alongWith(arm.setAngleSpeed(-10))
                        .until(rollergripper.beambreakTrigger.negate()));
//                        ejectCommand(0, -10, false))
    }

    public Command placeOnMidSequentially() {
        return new SequentialCommandGroup(
                arm.resetLengthCommand(),
                arm.moveToAngleCommand(MID_AUTO.setpoint).alongWith(
                        new WaitCommand(1).andThen(arm.moveToLengthCommand(MID_AUTO.setpoint)))
                        .until(()-> arm.armAtSetpoint(new Translation2d(0.85, Rotation2d.fromDegrees(174)))),
                rollergripper.ejectCommand().alongWith(arm.setAngleSpeed(-5)).until(rollergripper.beambreakTrigger.negate()),
                arm.resetLengthCommand().withTimeout(0.75)
        );
    }

    public Command placeOnLowCommand() {
        return arm.holdSetpointCommand(LOW.setpoint).withTimeout(0.65)
                .andThen(rollergripper.ejectCommand(0.2))
                .until(rollergripper.beambreakTrigger.negate().debounce(0.05));
    }

    public Command lockArmCommand() {
        return new ConditionalCommand(
                arm.lockArmCommand(CONE_LOCK),
                arm.lockArmCommand(LOCKED),
                rollergripper.beambreakTrigger
        );
    }

    public Command ejectCommand(double ejectOffset, double armOffset, boolean holdArm){
        return rollergripper.ejectCommand(ejectOffset).alongWith(arm.setAngleSpeed(armOffset))
                .until(rollergripper.beambreakTrigger.negate())
                .andThen(new ConditionalCommand(
                        arm.holdSetpointCommand(SHELF_RETRACTED.setpoint),
                        new InstantCommand(()-> {}),
                        ()-> holdArm));
    }

    // this command is used when the cuber needs to lean back, it moves the Arm, so they don't collide
    public Command adjustForShooterCommand(Command cuberCommand, BooleanSupplier canReturn) {
        return new SequentialCommandGroup(
                arm.holdSetpointCommand(CUBER_CHECKPOINT.setpoint).until(arm::armAtSetpoint),
                arm.holdSetpointCommand(CUBER.setpoint).alongWith(cuberCommand).until(cuberCommand::isFinished))
                .finallyDo((__)-> new WaitUntilCommand(canReturn).andThen(arm.lockArmWithSetpoint()).schedule());
    }

    public Command placeOnHeightCommand(double height) {
        return new SelectCommand(
                Map.of(
                        0.0, placeOnLowCommand(),
                        1.0, placeOnMidSequentially(),
                        2.0, new InstantCommand(() -> {})),
                // arm mechanics doesn't allow high cone placement in autonomous
                () -> height);
    }
}