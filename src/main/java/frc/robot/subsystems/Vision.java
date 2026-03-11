package frc.robot.subsystems;
  
import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

    
public class Vision extends SubsystemBase {

  private final PhotonCamera frontLeftCamera;
  private final PhotonCamera frontRightCamera;

  private final PhotonPoseEstimator frontLeftEstimator;
  private final PhotonPoseEstimator frontRightEstimator;
  private VisionSystemSim visionSim;
  private PhotonCameraSim frontLeftCamSim;
  private PhotonCameraSim frontRightCamSim;

  private final BiConsumer<Pose2d, Double> poseConsumer;

  //Callback to send pose estimates to the drivetrain
  public Vision(BiConsumer<Pose2d, Double> poseConsumer) {
    this.poseConsumer = poseConsumer;

  frontLeftCamera = new PhotonCamera(Constants.Vision.kFrontLeftCameraName);
  frontRightCamera = new PhotonCamera(Constants.Vision.kFrontRightCameraName);

  frontLeftEstimator = new PhotonPoseEstimator(
    Constants.Vision.kTagLayout,
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    Constants.Vision.kRobotToFrontLeftCamera);
  frontLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  
  frontRightEstimator = new PhotonPoseEstimator(
    Constants.Vision.kTagLayout,
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    Constants.Vision.kRobotToFrontRightCamera);
  frontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  

  /**
   * Creates a new Vision subsystem
   *  @param poseConsumer A method reference to add vision measurements (e.g., drivetrain::addVisionMeasurement)
   */
    // Sim
    if (Robot.isSimulation()) {
      //Create the vision simulation
      visionSim = new VisionSystemSim("main");
      //Add all the apriltags inside the tag layout as visible targets
      visionSim.addAprilTags(Constants.Vision.kTagLayout);
      //Create properties for sim cam
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(.35, .10);
      cameraProp.setFPS(60);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      
      frontLeftCamSim = new PhotonCameraSim(frontLeftCamera, cameraProp);
      visionSim.addCamera(frontLeftCamSim, Constants.Vision.kRobotToFrontLeftCamera);
      frontLeftCamSim.enableDrawWireframe(true);

      frontRightCamSim = new PhotonCameraSim(frontRightCamera, cameraProp);
      visionSim.addCamera(frontRightCamSim, Constants.Vision.kRobotToFrontRightCamera);
      frontRightCamSim.enableDrawWireframe(true);
    }
  }

  
  @Override
  public void periodic() {
    processCameraResult(frontLeftCamera, frontLeftEstimator, "FL");
    processCameraResult(frontRightCamera, frontRightEstimator, "FR");
    }
  /** 
    @param camera
    @param estimator
    @param prefix

  */
    private void processCameraResult(PhotonCamera camera, PhotonPoseEstimator estimator, String prefix) {
      PhotonPipelineResult result = camera.getLatestResult();
      SmartDashboard.putBoolean("Vision/" + prefix + "/Connected", camera.isConnected());
      SmartDashboard.putBoolean("Vision/" + prefix + "/Has Targets", result.hasTargets());

     
    if (result.hasTargets()) {
      SmartDashboard.putNumber("Vision/" + prefix + "/Target Count",
          result.getTargets().size());
      SmartDashboard.putNumber("Vision/" + prefix + "/Best Tag ID",
          result.getBestTarget().getFiducialId());
    }

    if (!result.hasTargets()) return;

    Optional<EstimatedRobotPose> estimatedPose = estimator.update(result);
    if (estimatedPose.isEmpty()) return;

    EstimatedRobotPose pose = estimatedPose.get();
    Pose2d pose2d = pose.estimatedPose.toPose2d();

    SmartDashboard.putNumber("Vision/" + prefix + "/Pose X",    pose2d.getX());
    SmartDashboard.putNumber("Vision/" + prefix + "/Pose Y",    pose2d.getY());
    SmartDashboard.putNumber("Vision/" + prefix + "/Timestamp", pose.timestampSeconds);
    

    Matrix<N3, N1> stdDevs = computeStdDevs(pose, estimator);

    SmartDashboard.putString("Vision/" + prefix + "/Std Devs",
        String.format("[%.2f, %.2f, %.2f]",
            stdDevs.get(0, 0), stdDevs.get(1, 0), stdDevs.get(2, 0)));

    poseConsumer.accept(pose2d, pose.timestampSeconds);
  }

  
 private Matrix<N3, N1> computeStdDevs(EstimatedRobotPose estimatedPose,
 PhotonPoseEstimator estimator) {
  Matrix<N3, N1> estStdDevs = Constants.Vision.kSingleTagStdDevs;
  int numTags = estimatedPose.targetsUsed.size();
  double avgDist = 0;

  for (var target : estimatedPose.targetsUsed) {
    var tagPose = estimator.getFieldTags().getTagPose(target.getFiducialId());
    if (tagPose.isPresent()) {
      avgDist =+ tagPose.get().toPose2d().getTranslation()
      .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
    }
  }

  if (numTags > 0) avgDist /= numTags;

  if (numTags > 1) {
    estStdDevs = Constants.Vision.kMultiTagStdDevs;
  }

  if (numTags == 1 && avgDist > 4) {
    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  } else {
    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
  }

  return estStdDevs;
 }

 public double getHubDistance() {
  for (PhotonCamera camera : new PhotonCamera[] {frontLeftCamera, frontRightCamera}) {
    PhotonPipelineResult result = camera.getLatestResult();
    if (!result.hasTargets()) continue;

    for (var target: result.getTargets()) {
      int id = target.getFiducialId();
      if (id == 10 || id == 25) {
        double distance = target.getBestCameraToTarget()
        .getTranslation().getNorm();
        SmartDashboard.putNumber("Shooter/Distance (m)", distance);
        return distance;
      }
    }
  }

  SmartDashboard.putNumber("Shooter/Hub Distance (m)", -1);
  return 3;
 }

 public PhotonCamera getCamera() {return frontLeftCamera;}
 public PhotonCamera getFrontLeftCamera() {return frontLeftCamera;}
 public PhotonCamera getFrontRightCamera() {return frontRightCamera;}

 public void simulationPeriodic(Pose2d robotSimPose) {
  if (visionSim != null) visionSim.update(robotSimPose); 
 }

 public void resetSimPose(Pose2d pose) {
  if (Robot.isSimulation() && visionSim !=null) visionSim.resetRobotPose(pose); 
 }
}
