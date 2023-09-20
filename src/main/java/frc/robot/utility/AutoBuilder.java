 package frc.robot.utility;

 import com.pathplanner.lib.PathPlannerTrajectory;
 import com.pathplanner.lib.PathPoint;
 import com.pathplanner.lib.auto.PIDConstants;
 import com.pathplanner.lib.auto.SwerveAutoBuilder;
 import edu.wpi.first.math.controller.PIDController;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.button.Trigger;
 import frc.robot.subsystems.swerve.Swerve;

 import java.util.HashMap;

 import static frc.robot.Constants.SwerveConstants.*;

 public class AutoBuilder {
     private Swerve swerve;

     private final SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
             swerve::getPose2d, (__)->{},
             kSwerveKinematics,
             new PIDConstants(kp_TRANSLATION, 0, kd_TRANSLATION),
             new PIDConstants(kp_ANGLE, 0, kd_ANGLE),
             swerve::setModulesStates,
             new HashMap<>(),
             true, // WTFFF
             swerve
     );

     public AutoBuilder(Swerve swerve){
         this.swerve = swerve;
     }

     public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
         return swerve.straightenModulesCommand().andThen(swerveAutoBuilder.fullAuto(traj));
     }

     public Command turnToAngleCommand(double setpoint) {
         return swerve.tankDriveCommand(
                 () -> 0,
                 () -> new PIDController(kp_ANGLE, 0, kd_ANGLE).calculate(swerve.getOdometryRotation2d().getDegrees(), setpoint),
                 false)
                 .until(new Trigger(()-> Math.abs(swerve.getOdometryRotation2d().getDegrees() - setpoint) < 1.5).debounce(0.15));
     }

    private static PathPoint getPathpoint(PathPoint point){
        return new PathPoint(new Translation2d(point.position.getX(), -point.position.getY()), point.heading.times(-1), Rotation2d.fromDegrees(360).minus(point.holonomicRotation));
    }

    public static PathPoint getPathpoint(Translation2d position, Rotation2d heading, Rotation2d holonomicRotation){
        return getPathpoint(new PathPoint(position, heading, holonomicRotation));
    }
 }
