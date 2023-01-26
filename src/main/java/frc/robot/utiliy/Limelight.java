package frc.robot.utiliy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import java.io.IOException;
import java.util.function.BiConsumer;

public class Limelight {
  private final PhotonCamera ll = new PhotonCamera("Limelight");
  private final AprilTagFieldLayout fieldLayout;

  {
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
      throw new RuntimeException(e);
    }
  }

  public void setDriverMode(boolean isDriverMode){
    ll.setDriverMode(isDriverMode);
  }

  public void setLEDMode(VisionLEDMode ledMode){
    ll.setLED(ledMode);
  }

  public enum PipelineType{
    AprilTags(0),
    ReflectiveTape(1),
    Cone(2),
    Cube(3);

    private int pipeline;

    PipelineType(int pipeline){
      this.pipeline = pipeline;
    }
  }

  public void setPipeline(PipelineType pipeline){
    ll.setPipelineIndex(pipeline.pipeline);
  }

  public boolean updateFromAprilTagPose(BiConsumer<Pose2d, Double> toUpdate) {
    var result = ll.getLatestResult();
    if (!result.hasTargets()) return false;

    var id = result.getBestTarget().getFiducialId();
    if (id == -1) return false;

    var tag = fieldLayout.getTagPose(id);
    if (tag.isEmpty()) return false;

    toUpdate.accept(
          tag.get().plus(result.getBestTarget().getBestCameraToTarget()).toPose2d(),
          result.getTimestampSeconds());

    return true;
  }
}
