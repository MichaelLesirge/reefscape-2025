package frc.robot.commands.tagFollowing;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.utility.VirtualSubsystem;

public class TargetFollowing extends VirtualSubsystem {

  private final AprilTagVision vision;
  private final Supplier<Pose2d> robotPoseSupplier;

  private IntSupplier tagToFollow;
  private boolean hasTagInView = false;

  public TargetFollowing(AprilTagVision vision, Supplier<Pose2d> robotPoseSupplier) {
    this.vision = vision;
    this.robotPoseSupplier = robotPoseSupplier;
    
  }

  public void setTagToFollow(IntSupplier tagToFollow) {
    this.tagToFollow = tagToFollow;
  }

  @Override
  public void periodic() {
    vision
        .getTransformToTag(tagToFollow.getAsInt())
        .ifPresentOrElse(
            t -> {
              Pose3d robotPose = TagFollowingUtil.toPose3d(robotPoseSupplier.get());

              Pose3d cameraPose = robotPose.plus(t.robotToCamera());

              Pose3d tagPose = cameraPose.plus(t.cameraToTarget());

              hasTagInView = true;

              Logger.recordOutput("TagFollowing/FollowedTag", new Pose3d[] { tagPose });
              Logger.recordOutput("TagFollowing/UsedCamera", cameraPose);
            }, () -> {
              hasTagInView = false;
              Logger.recordOutput("TagFollowing/FollowedTag", new Pose3d[] {});
              Logger.recordOutput("TagFollowing/UsedCamera", Pose3d.kZero);
            });
  }

  public boolean hasTagInView() {
    return hasTagInView;
  }
}
