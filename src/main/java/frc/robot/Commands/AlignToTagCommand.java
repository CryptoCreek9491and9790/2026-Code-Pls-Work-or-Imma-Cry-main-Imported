package frc.robot.Commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToTagCommand extends Command {
    //Tag IDs to align to - add or change as needed
    private static final int[] TARGET_IDS = {10, 25};

    //How close you want to stop from the tag(meters)
    private static final double DESIRED_RANGE_M = .7;
    private static final double RANGE_DEADBAND_M = .1;

    //PID for yaw correction (turning to face the tag)
    //Tune kP:increase if turning is too slow, decrease if it oscillates
    private final PIDController yawController = new PIDController(.2, 0, 0);
    private final PIDController rangeController = new PIDController(.5, 0, .05);

    private final DriveSubsystem drivetrain;
    private final PhotonCamera frontRightCamera;
    private final PhotonCamera frontLeftCamera;
    private final AprilTagFieldLayout fieldLayout;
    
    public AlignToTagCommand(DriveSubsystem drivetrain, PhotonCamera frontLeftCamera, PhotonCamera frontRightCamera, AprilTagFieldLayout fieldLayout) {
        this.drivetrain = drivetrain;
        this.frontLeftCamera = frontLeftCamera;
        this.frontRightCamera = frontRightCamera;
        this.fieldLayout = fieldLayout;
        addRequirements(drivetrain);

        yawController.setTolerance(5); //Degrees
        rangeController.setTolerance(RANGE_DEADBAND_M);
    }

    @Override
    public void initialize() {
        yawController.reset();
        rangeController.reset();
    }

    @Override
    public void execute() {
        CameraTarget found = findTarget(frontLeftCamera);
        if (found == null) found = findTarget(frontRightCamera);

        if (found == null) {
            stop();
            SmartDashboard.putBoolean("Align/Target Visible", false);
            SmartDashboard.putString("Align/Camera used", "none");
            return;
        }

        SmartDashboard.putBoolean("Align/Target Visible", true);
        SmartDashboard.putNumber("Align/Matched Tag ID",  found.id);
        SmartDashboard.putString("Align/Camera Used",     found.cameraName);

        double turn = yawController.calculate(found.yaw, 0);
        turn = Math.max(-.5, Math.min(.5, turn));

        //Calculate drive speeds from field pose
        double xSpeed = 0;
        double ySpeed = 0;

        var tagPose = fieldLayout.getTagPose(found.id);
        if (tagPose.isPresent()) {
            Pose2d robotPose = drivetrain.getPose();

            //Vector pointing from robot to tag
            Translation2d robotToTag = tagPose.get().toPose2d().getTranslation()
                .minus(robotPose.getTranslation());

            double distance = robotToTag.getNorm();
            double rangeError = distance - DESIRED_RANGE_M;

            if (Math.abs(rangeError) > RANGE_DEADBAND_M && distance > .01) {
                //Normalize the direction, scale by PID output
                Translation2d direction = robotToTag.div(distance);
                double speed = -rangeController.calculate(distance, DESIRED_RANGE_M);
                speed = Math.max(-.6, Math.min(.6, speed));

                xSpeed = direction.getX() * speed;
                ySpeed = direction.getY() * speed;
                
            }
            
        }

        drivetrain.drive(xSpeed, ySpeed, turn, true);
    }

    @Override
    public void end(boolean interrupted) {
        stop();
        yawController.reset();
        rangeController.reset();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    private void stop() {
        drivetrain.drive(0, 0, 0, false);
    }

    private CameraTarget findTarget(PhotonCamera camera) {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) return null;

        for (var target : result.getTargets()) {
            for (int id : TARGET_IDS) {
                if (target.getFiducialId() == id) {
                    return new CameraTarget(id, target.getYaw(), camera.getName());
                }
            }
        }
        return null;
    }

    private static class CameraTarget {
        final int id;
        final double yaw;
        final String cameraName;

        CameraTarget(int id, double yaw, String cameraName) {
            this.id = id;
            this.yaw = yaw;
            this.cameraName = cameraName;
        }
    }
}
