package frc.robot.subsystems;
  
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

    
public class Vision extends SubsystemBase {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private final AprilTagFieldLayout aprilTagFieldLayout;

  //Callback to send pose estimates to the drivetrain
  private final BiConsumer<Pose2d, Double> poseConsumer;

  /**
   * Creates a new Vision subsystem
   *  @param poseConsumer A method reference to add vision measurements (e.g., drivetrain::addVisionMeasurement)
   */
  public Vision (BiConsumer<Pose2d, Double> poseConsumer) {
    this.camera = new PhotonCamera("maincam");
    this.poseConsumer = poseConsumer;

    //Load the AprilTag Field Layout for 2026
    this.aprilTagFieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

    //Create the PhotonPoseEstimator
    this.photonEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
       Constants.Vision.kRobotToCam);

      //Set the fallback pose strategy for single tag detections
      photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  
  @Override
  public void periodic() {
    //Get the latest result from the camera
    PhotonPipelineResult result = camera.getLatestResult();

    //Update dashboard with camera status
    SmartDashboard.putBoolean("Camera Connected", camera.isConnected());
    SmartDashboard.putBoolean("Has Targets", result.hasTargets());

    if (result.hasTargets()) {
      SmartDashboard.putNumber("Target Count", result.getTargets().size());
      SmartDashboard.putNumber("Best Target ID", result.getBestTarget().getFiducialId());
    }
  
    //Try to get an estimated robot pose
  Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose();

    if(estimatedPose.isPresent()) {
      EstimatedRobotPose pose = estimatedPose.get();

      //Log the Vision Pose to dashboard
      SmartDashboard.putNumber("Vision Pose X", pose.estimatedPose.toPose2d().getX());
      SmartDashboard.putNumber("Vision Pose Y", pose.estimatedPose.toPose2d().getY());
      SmartDashboard.putNumber("Vision Timestamp", pose.timestampSeconds);

      //Determine standard deviation based on number of tags and distance
      var stdDevs = getEstimationStdDevs(pose);

      //Add the vision measurement to the pose estimator with appropriate standard deviations
      poseConsumer.accept(
        pose.estimatedPose.toPose2d(),
         pose.timestampSeconds);

      SmartDashboard.putString("Vision Std Devs", stdDevs.toString());
    }
  }

  //Get the estimated global pose from PhotonPoseEstimator
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var result = camera.getLatestResult();
    if (!result.hasTargets()) {
      return Optional.empty();
    }
    return photonEstimator.update(result);
  }
  
  private String getEstimationStdDevs(EstimatedRobotPose estimatedPose) {
    var estStdDevs = Constants.Vision.kSingleTagStdDevs;
    int numTags = estimatedPose.targetsUsed.size();
    double avgDist = 0;

    for (var target : estimatedPose.targetsUsed) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(target.getFiducialId());
      if (tagPose.isPresent()) {
        avgDist += tagPose.get().toPose2d().getTranslation()
        .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
      }
    }

    if (numTags > 0) {
      avgDist /= numTags;
    }

    if(numTags > 1) {
      estStdDevs = Constants.Vision.kMultiTagStdDevs;
    }

    //Increase standard deviations based on distance
    if (numTags ==1 && avgDist > 4) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else{
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    }

    return String.format("Tags: %d, Dist: %.2f m, StdDevs: [%.2f, %.2f, %.2f]", numTags, avgDist, estStdDevs.get(0,0), estStdDevs.get(1,0), estStdDevs.get(2,0));
  }

  //Gets the PhotonCamera instance
  public PhotonCamera getCamera() {
    return camera;
  }
}
