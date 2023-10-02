 package frc.robot.utility;

 import com.pathplanner.lib.PathConstraints;
 import com.pathplanner.lib.PathPlanner;
 import com.pathplanner.lib.PathPlannerTrajectory;
 import com.pathplanner.lib.PathPoint;
 import com.pathplanner.lib.auto.PIDConstants;
 import com.pathplanner.lib.auto.SwerveAutoBuilder;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.PrintCommand;
 import frc.robot.subsystems.swerve.Swerve;

 import java.util.HashMap;
 import java.util.List;

 import static frc.robot.Constants.SwerveConstants.*;

 public class SwerveAuto {
     private Swerve swerve;

     private final SwerveAutoBuilder swerveAutoBuilder;

     public SwerveAuto(Swerve swerve){
         this.swerve = swerve;

         swerveAutoBuilder = new SwerveAutoBuilder(
                 swerve::getPose2d, (__)-> {},
                 kSwerveKinematics,
                 new PIDConstants(0, 0, 0),
                 new PIDConstants(kp_ANGLE, 0, kd_ANGLE),
                 swerve::setModulesStates,
                 new HashMap<>(),
                 true,
                 swerve
         );
     }

     public Command followTrajectoryCommand(String trajName) {
         PathPlannerTrajectory traj = PathPlanner.loadPath(trajName, new PathConstraints(2, 2));
         if (traj != null) return swerve.straightenModulesCommand().andThen(swerveAutoBuilder.fullAuto(traj));
         return new PrintCommand("null path");
     }

     public Command generatePath(List<PathPoint> points){
//         return swerve.setOdometryPositionCommand(new Pose2d(new Translation2d(0, 0), swerve.getGyroRotation2d())).andThen(
                 return swerveAutoBuilder.followPath(
                 PathPlanner.generatePath(new PathConstraints(4, 3), false, points));
     }

    private static PathPoint getPathpoint(PathPoint point){
        return new PathPoint(new Translation2d(point.position.getX(), -point.position.getY()), point.heading.times(-1), Rotation2d.fromDegrees(360).minus(point.holonomicRotation));
    }

    public static PathPoint getPathpoint(Translation2d position, double heading, double holonomicRotation){
        return getPathpoint(new PathPoint(position, Rotation2d.fromDegrees(heading), Rotation2d.fromDegrees(holonomicRotation)));
    }
 }
