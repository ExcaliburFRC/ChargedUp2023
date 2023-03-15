package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import static frc.robot.Constants.ArmConstants.Setpoints.*;

public class Superstructure extends SubsystemBase {
	public final Arm arm = new Arm();
	public final RollerGripper rollerGripper = new RollerGripper();

	public Superstructure() {
		rollerGripper.setDefaultCommand(rollerGripper.holdConeCommand());

		arm.setDefaultCommand(arm.fadeArmCommand());
	}

	public Command intakeFromShelfCommand() {
		return arm.resetLengthCommand().andThen(
//          new InstantCommand(()-> Shuffleboard.selectTab("armCamera")),
				rollerGripper.intakeCommand().alongWith(arm.holdSetpointCommand(SHELF_EXTENDED.setpoint))
						.until(rollerGripper.beambreakTrigger),
//                new InstantCommand(() -> Shuffleboard.selectTab("Swerve")),
				arm.holdSetpointCommand(SHELF_RETRACTED.setpoint).withTimeout(0.5));
	}

	public Command placeOnHighCommand(Trigger release) {
		return new SequentialCommandGroup(
				arm.holdSetpointCommand(HIGH_CHECKPOINT.setpoint).withTimeout(1),
				arm.holdSetpointCommand(HIGH.setpoint))
				.raceWith(new WaitUntilCommand(release))
				.andThen(arm.fadeArmCommand().alongWith(rollerGripper.ejectCommand(0.035)))
				.until(rollerGripper.beambreakTrigger.negate().debounce(1.5));
	}

	public Command placeOnMidCommand(Trigger release) {
		return arm.holdSetpointCommand(MID.setpoint)
				.raceWith(new WaitUntilCommand(release))
				.andThen(arm.fadeArmCommand().alongWith(rollerGripper.ejectCommand()))
				.until(rollerGripper.beambreakTrigger.negate().debounce(0.2));
	}

	public Command placeOnLowCommand() {
		return arm.holdSetpointCommand(LOW.setpoint)
				.raceWith(new WaitCommand(0.35))
				.andThen(rollerGripper.ejectCommand(0.2))
				.until(rollerGripper.beambreakTrigger.negate().debounce(0.05));
	}

	public Command lockArmCommand(){
		return arm.lockArmCommand(rollerGripper.beambreakTrigger);
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
