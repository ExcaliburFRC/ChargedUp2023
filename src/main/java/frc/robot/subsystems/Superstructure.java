package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ArmConstants.Setpoints.*;
import static frc.robot.utility.Colors.ORANGE;

public class Superstructure {
    public final Arm arm = new Arm();
    public final Rollergripper rollergripper = new Rollergripper();

    public Superstructure() {}

    public Command intakeFromShelfCommand() {
        return new SequentialCommandGroup(
                arm.moveToLengthCommand(INTAKE_CHECKPOINT.setpoint),
                new ParallelRaceGroup(
                        rollergripper.intakeCommand(),
                        arm.holdSetpointCommand(SHELF_EXTENDED.setpoint),
                        LEDs.getInstance().applyPatternCommand(LEDs.LEDPattern.BLINKING, ORANGE.color)))
                .finallyDo((__)-> arm.holdSetpointCommand(SHELF_RETRACTED.setpoint).alongWith(rollergripper.holdConeCommand()).schedule());
    }

    public Command placeOnHighCommand(Trigger release) {
        return new SequentialCommandGroup(
                arm.holdSetpointCommand(BUMPER.setpoint).withTimeout(0.3),
                arm.holdSetpointCommand(HIGH_CHECKPOINT.setpoint).withTimeout(1.25),
                arm.holdSetpointCommand(HIGH.setpoint).until(release)).unless(rollergripper.beambreakTrigger.negate())
//                arm.osscilateArmCommand(HIGH.setpoint, 7).until(release.negate()))
                .finallyDo((__)-> ejectCommand(0, 0, true).schedule());
    }

    public Command placeOnMidCommand(BooleanSupplier release) {
                return arm.holdSetpointCommand(MID.setpoint).until(release).unless(rollergripper.beambreakTrigger.negate())
                .finallyDo((__)-> ejectCommand(0, -5, false).schedule());
    }

    public Command placeOnMidSequentially() {
        return arm.resetLengthCommand()
                .andThen(arm.moveToAngleCommand(MID.setpoint))
                .alongWith(new WaitCommand(1).andThen(arm.moveToLengthCommand(MID.setpoint)))
                .until(() -> arm.armAtSetpoint(MID.setpoint)).andThen(ejectCommand(0, -5, false));
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
                .finallyDo((__)->
                        new WaitUntilCommand(canReturn).andThen(arm.lockArmWithSetpoint()).schedule());
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