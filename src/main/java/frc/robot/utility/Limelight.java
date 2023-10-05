package frc.robot.utility;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.PhotonCamera;

import java.io.IOException;
import java.util.function.BiConsumer;

public class Limelight {
  PhotonCamera ll = new PhotonCamera("Limelight");
  AprilTagFieldLayout fieldLayout;

//  public static final ShuffleboardTab armCameraTab = Shuffleboard.getTab("armCamera");
//  public static final ShuffleboardTab intakeCameraTab = Shuffleboard.getTab("intakeCamera");

  public Limelight() {
    ll.setDriverMode(true);

//    armCameraTab.addCamera("lifeCam", "Microsoft_LifeCam_HD-3000", "http://10.67.38.11:1182/stream.mjpg?1678398420660"); //.withPosition(2, 0).withSize(11, 6);
//    intakeCameraTab.addCamera("limelight", "Limelight", "http://10.67.38.11:5800/#/dashboard");
  }

  {
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
      throw new RuntimeException(e);
    }
  }

  public boolean updateFromAprilTagPose(BiConsumer<Pose2d, Double> toUpdate) {
    var result = ll.getLatestResult();
    if (!result.hasTargets()) return false;

    var id = result.getBestTarget().getFiducialId();
    if (id == -1) return false;

    var tag = fieldLayout.getTagPose(id);
    if (tag.isEmpty()) return false;

    toUpdate.accept(tag.get().plus(result.getBestTarget().getBestCameraToTarget()).toPose2d(), result.getTimestampSeconds());
    return true;
  }
}
