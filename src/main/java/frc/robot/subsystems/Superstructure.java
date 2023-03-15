package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import static frc.robot.Constants.ArmConstants.Setpoints.*;

public class Superstructure extends SubsystemBase {
	public final Arm arm = new Arm();
	public final Rollergripper rollergripper = new Rollergripper();

	public Superstructure() {
		rollergripper.setDefaultCommand(rollergripper.holdConeCommand());

		arm.setDefaultCommand(arm.fadeArmCommand());
	}

	public Command intakeFromShelfCommand() {
		return arm.resetLengthCommand().andThen(
//          new InstantCommand(()-> Shuffleboard.selectTab("armCamera")),
				rollergripper.intakeCommand().alongWith(arm.holdSetpointCommand(SHELF_EXTENDED.setpoint))
						.until(rollergripper.beambreakTrigger),
//                new InstantCommand(() -> Shuffleboard.selectTab("Swerve")),
				arm.holdSetpointCommand(SHELF_RETRACTED.setpoint).withTimeout(0.5));
	}

	public Command placeOnHighCommand(Trigger release) {
		return new SequentialCommandGroup(
				arm.holdSetpointCommand(HIGH_CHECKPOINT.setpoint).withTimeout(1),
				arm.holdSetpointCommand(HIGH.setpoint))
				.raceWith(new WaitUntilCommand(release))
				.andThen(arm.fadeArmCommand().alongWith(rollergripper.ejectCommand(0.035)))
				.until(rollergripper.beambreakTrigger.negate().debounce(1.5));
	}

	public Command placeOnMidCommand(Trigger release) {
		return arm.holdSetpointCommand(MID.setpoint)
				.raceWith(new WaitUntilCommand(release))
				.andThen(arm.fadeArmCommand().alongWith(rollergripper.ejectCommand()))
				.until(rollergripper.beambreakTrigger.negate().debounce(0.2));
	}

	public Command placeOnLowCommand() {
		return arm.holdSetpointCommand(LOW.setpoint)
				.raceWith(new WaitCommand(0.35))
				.andThen(rollergripper.ejectCommand(0.2))
				.until(rollergripper.beambreakTrigger.negate().debounce(0.05));
	}

	public Command lockArmCommand(){
		return arm.lockArmCommand(rollergripper.beambreakTrigger);
	}

	@Deprecated
	public Command switchCommand(double height) {
		if (height == Constants.IntakeConstants.LOW_RPM)
			return placeOnLowCommand(); //TODO: find minimal time for cone placement
		if (height == Constants.IntakeConstants.MID_RPM) return placeOnMidCommand(new Trigger(() -> true).debounce(5));
		if (height == Constants.IntakeConstants.HIGH_RPM)
			return placeOnHighCommand(new Trigger(() -> true).debounce(5));
		return new InstantCommand(() -> {
		});
	}

}
