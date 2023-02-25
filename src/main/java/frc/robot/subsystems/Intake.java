package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.utiliy.ToggleCommand;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, kBrushless);

    private final DoubleSolenoid intakePiston = new DoubleSolenoid(REVPH, INTAKE_FWD_CHANNEL, INTAKE_REV_CHANNEL);
    private final DoubleSolenoid ejectPiston = new DoubleSolenoid(REVPH, EJECT_FWD_CHANNEL, EJECT_REV_CHANNEL);

    public Intake() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor.clearFaults();
    }

    public Command openPistonCommand() {
        return new InstantCommand(() -> {
            intakePiston.set(DoubleSolenoid.Value.kForward);
        }, this);
    }

    public Command closePistonCommand() {
        return new InstantCommand(() -> {
            intakePiston.set(DoubleSolenoid.Value.kReverse);
        },this);
    }

    public Command intakeCommand(double intakeSpeed){
        return new StartEndCommand(
              ()-> {
                  intakePiston.set(DoubleSolenoid.Value.kForward);
                  intakeMotor.set(intakeSpeed);
              },
              ()-> {
                  intakePiston.set(DoubleSolenoid.Value.kReverse);
                  intakeMotor.stopMotor();
              }, this
        );
    }

    private Command ejectCubeCommand(){
        return new InstantCommand((()-> ejectPiston.set(DoubleSolenoid.Value.kForward)));
    }

    private Command retractPistonCommand(){
        return new InstantCommand((()-> ejectPiston.set(DoubleSolenoid.Value.kReverse)));
    }

    public Command pulseMotorCommand(){
        return new RunCommand(()-> intakeMotor.set(-0.3)).withTimeout(0.1);
    }

    public Command shootCubeCommand(int height){
        switch (height){
            case 1:
                return new ToggleCommand(
                      Commands.repeatingSequence(
                      new RunCommand(()-> intakeMotor.set(-0.075), this).withTimeout(0.3), pulseMotorCommand())
                      .finallyDo((__)-> intakeMotor.stopMotor()),
                      new InstantCommand(intakeMotor::stopMotor).alongWith(retractPistonCommand()));
            case 2:
                return new ToggleCommand(
                      Commands.runEnd(()-> intakeMotor.set(-0.3), intakeMotor::stopMotor, this)
                      .alongWith(new WaitCommand(0.2).andThen(ejectCubeCommand())),
                      new InstantCommand(intakeMotor::stopMotor).alongWith(retractPistonCommand()));
          case 3:
                return new ToggleCommand(
                      Commands.runEnd(()-> intakeMotor.set(-0.54), intakeMotor::stopMotor, this)
                      .alongWith(new WaitCommand(0.4).andThen(ejectCubeCommand())),
                      new InstantCommand(intakeMotor::stopMotor).alongWith(retractPistonCommand()));
          default:
              return new InstantCommand(()-> {});
        }
    }

    // top - 54
    // middle - 30
    // low - 5
}
