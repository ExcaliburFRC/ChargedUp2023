package frc.robot.Utiliy;

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

    toUpdate.accept(tag.get().toPose2d(), result.getTimestampSeconds());
    return true;
  }
}
