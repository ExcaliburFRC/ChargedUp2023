package frc.robot.utiliy;

import edu.wpi.first.wpilibj2.command.*;

public class ToggleCommand extends FunctionalCommand {
  /**
   * Creates a new StartEndCommand. Will run the given runnables when the command starts and when it
   * ends.
   *
   * @param onInit       the Runnable to run on command init
   * @param onEnd        the Runnable to run on command end
   * @param requirements the subsystems required by this command
   */
  public ToggleCommand(Command onInit, Command onEnd, Subsystem... requirements) {
    super(
          onInit::schedule,
          () -> {
          },
          (__) -> onEnd.schedule(),
          () -> false,
          requirements);
  }
}
