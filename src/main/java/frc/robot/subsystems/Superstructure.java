package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.Setpoints;

import static frc.robot.Constants.ArmConstants.Setpoints.*;

public class Superstructure extends SubsystemBase {
  public final Arm arm = new Arm();
  public final RollerGripper rollerGripper = new RollerGripper();

  public Superstructure() {
    rollerGripper.setDefaultCommand(rollerGripper.holdConeCommand());

    arm.setDefaultCommand(
          new ConditionalCommand(
                // calibrate arm
                arm.resetLengthCommand(),
                // lock / close arm
                new ConditionalCommand(
                      // close arm
                      arm.closeArmCommand(),
                      // lock arm
                      new ConditionalCommand(
                            new InstantCommand(()->{}),
                            arm.lockArmCommand(),
                            arm.armLockedTrigger),
                      // whether were holding a cone
                      rollerGripper.beambreakTrigger),
                // whether arm is calibrated
                ()-> arm.getArmLength() < 0.1));
  }

  public Command intakeFromShelfCommand() {
    return arm.resetLengthCommand().andThen(
          new InstantCommand(()-> Shuffleboard.selectTab("cameraTab")),
          new ParallelCommandGroup(
          rollerGripper.intakeCommand(),
          arm.holdSetpointCommand(SHELF_EXTENDED.setpoint)))
          .until(rollerGripper.beambreakTrigger)
          .andThen(
                new InstantCommand(()-> Shuffleboard.selectTab("Swerve")),
                arm.holdSetpointCommand(SHELF_RETRACTED.setpoint).withTimeout(0.5)
          );
  }

  public Command placeOnHighCommand(Trigger release) {
    return new SequentialCommandGroup(
          arm.holdSetpointCommand(HIGH_CHECKPOINT.setpoint).withTimeout(1),
          arm.holdSetpointCommand(HIGH.setpoint))
          .raceWith(new WaitUntilCommand(release))
          .andThen(arm.lowerArmCommand().alongWith(
                            rollerGripper.ejectCommand(0.035)))
          .until(rollerGripper.beambreakTrigger.negate().debounce(1.5));
  }

  public Command placeOnMidCommand(Trigger release) {
    return arm.holdSetpointCommand(MID.setpoint)
          .raceWith(new WaitUntilCommand(release))
          .andThen(arm.lowerArmCommand().alongWith(
                rollerGripper.ejectCommand()))
          .until(rollerGripper.beambreakTrigger.negate().debounce(0.2));
  }

  public Command placeOnLowCommand() {
    return arm.holdSetpointCommand(LOW.setpoint)
          .raceWith(new WaitCommand(0.3))
          .andThen(rollerGripper.ejectCommand(0.2))
          .until(rollerGripper.beambreakTrigger.negate().debounce(0.05));
  }

  public Command initArmCommand(Setpoints setpoint) {
          return Commands.parallel(
          arm.resetLengthCommand(),
          Commands.waitSeconds(1).andThen(arm.moveToAngleCommand(setpoint.setpoint)))
                .unless(rollerGripper.beambreakTrigger.negate());
  }

  public Command switchCommand(double height){
    if (height == Constants.IntakeConstants.LOW_RPM) return placeOnLowCommand(); //TODO: find minimal time for cone placement
    if (height == Constants.IntakeConstants.MID_RPM) return placeOnMidCommand(new Trigger(()-> true).debounce(5));
    if (height == Constants.IntakeConstants.HIGH_RPM) return placeOnHighCommand(new Trigger(()-> true).debounce(5));
    return new InstantCommand(()->{});
  }

}
