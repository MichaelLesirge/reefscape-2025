package frc.robot.commands.tagFollowing;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.controllers.HeadingController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.Camera.TrackedTarget;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AimAtTag extends Command {

  // Very bad code but ehh
  public static Supplier<Translation2d> drivingTranslationSupplier = () -> Translation2d.kZero;
  public static BooleanSupplier fieldRelativeDrivingSupplier = () -> false;

  private static final Rotation2d TARGET_HEADING_OFFSET = Rotation2d.k180deg;
  private static final int MEDIAN_FILTER_SIZe = 10;

  private final Drive drive;
  private final AprilTagVision vision;

  private final IntSupplier tagToFollow;

  private final MedianFilter targetHeadingFilter = new MedianFilter(MEDIAN_FILTER_SIZe);
  private final HeadingController headingController;

  public AimAtTag(AprilTagVision vision, Drive drive, IntSupplier tagToFollow) {
    this.vision = vision;
    this.drive = drive;
    this.tagToFollow = tagToFollow;

    this.headingController = new HeadingController(drive);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    headingController.reset();
    targetHeadingFilter.reset();
    headingController.setGoalToCurrentHeading();
  }

  @Override
  public void execute() {
    vision
        .getTransformToTag(tagToFollow.getAsInt())
        .filter(TrackedTarget::isGoodPoseAmbiguity)
        .ifPresent(
            t -> {
              Pose3d robotPose = TagFollowingUtil.toPose3d(drive.getRobotPose());

              Pose3d cameraPose = robotPose.plus(t.robotToCamera());

              Pose3d tagPose = cameraPose.plus(t.cameraToTarget());

              Rotation2d targetHeading =
                  tagPose
                      .getTranslation()
                      .minus(robotPose.getTranslation())
                      .toTranslation2d()
                      .getAngle()
                      .plus(TARGET_HEADING_OFFSET);

              Logger.recordOutput("TagFollowing/Aim/TargetHeading", targetHeading.getDegrees());

              Logger.recordOutput("TagFollowing/FollowedTag", tagPose);
              Logger.recordOutput("TagFollowing/UsedCamera", cameraPose);

              Rotation2d meanTargetHeading =
                  new Rotation2d(targetHeadingFilter.calculate(targetHeading.getRadians()));

              SmartDashboard.putNumber(
                  "Target Heading", ((-targetHeading.getDegrees() + 360) % 360));

              headingController.setGoal(meanTargetHeading);
            });

    Translation2d drivingTranslation = drivingTranslationSupplier.get();
    drive.setRobotSpeeds(
        new ChassisSpeeds(
            drivingTranslation.getX(), drivingTranslation.getY(), headingController.calculate()),
        fieldRelativeDrivingSupplier.getAsBoolean());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
