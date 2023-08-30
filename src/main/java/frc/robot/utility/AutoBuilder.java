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
 import frc.robot.swerve.Swerve;

 import java.util.HashMap;

 import static frc.robot.Constants.SwerveConstants.*;

 public class AutoBuilder {
     private Swerve swerve;

     private final SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
             swerve::getPose2d, swerve::setPose2d,
             kSwerveKinematics,
             new PIDConstants(0, 0, 0),
             new PIDConstants(kp_Theta, 0, kd_Theta),
             swerve::setModulesStates,
             new HashMap<>(),
             true,
             swerve
     );

     public AutoBuilder(Swerve swerve){
         this.swerve = swerve;
     }

     public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
         return swerve.resetModulesCommand().andThen(swerveAutoBuilder.fullAuto(traj));
     }

     public Command turnToAngleCommand(double setpoint) {
         return swerve.driveSwerveCommand(
                 () -> 0, () -> 0,
                 () -> new PIDController(kp_Theta, 0, kd_Theta).calculate(swerve.getOdometryRotation().getDegrees(), setpoint),
                 () -> false)
                 .until(new Trigger(()-> Math.abs(swerve.getOdometryRotation().getDegrees() - setpoint) < 1.5).debounce(0.15));
     }

    public static PathPoint getPathpoint(PathPoint point){
        return new PathPoint(new Translation2d(point.position.getX(), -point.position.getY()), point.heading.times(-1), Rotation2d.fromDegrees(360).minus(point.holonomicRotation));
    }

    public static PathPoint getPathpoint(Translation2d position, Rotation2d heading, Rotation2d holonomicRotation){
        return getPathpoint(new PathPoint(position, heading, holonomicRotation));
    }
 }
