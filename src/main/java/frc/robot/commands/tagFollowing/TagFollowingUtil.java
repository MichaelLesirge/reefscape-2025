package frc.robot.commands.tagFollowing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class TagFollowingUtil {

  public static final double ELEVATOR_HEIGHT_OFF_GROUND = Units.inchesToMeters(12);
  public static final double WRIST_LENGTH = Units.inchesToMeters(12);

  public static Pose3d toPose3d(Pose2d pose) {
    return new Pose3d(
        pose.getX(), pose.getY(), 0, new Rotation3d(0, 0, pose.getRotation().getRadians()));
  }

  public record TagFollowingSuperstructureState(double elevatorHeight, Rotation2d wristAngle) {
  }

  public static TagFollowingSuperstructureState getSuperstructureState(Pose3d tagPose) {
    final Rotation2d wristAngle = new Rotation2d(tagPose.getRotation().getY());
    final double elevatorHeight = tagPose.getTranslation().getZ() - WRIST_LENGTH * wristAngle.getSin() - ELEVATOR_HEIGHT_OFF_GROUND;
    return new TagFollowingSuperstructureState(elevatorHeight, wristAngle);
  }
}
