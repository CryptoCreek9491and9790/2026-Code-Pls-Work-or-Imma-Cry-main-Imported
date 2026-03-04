package frc.Commands;

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
    private static final double RANGE_DEADBAND_M = .3;

    //PID for yaw correction (turning to face the tag)
    //Tune kP:increase if turning is too slow, decrease if it oscillates
    private final PIDController yawController = new PIDController(.2, 0, 1);
    private final PIDController rangeController = new PIDController(.5, 0, .05);

    private final DriveSubsystem drivetrain;
    private final PhotonCamera camera;
    private final AprilTagFieldLayout fieldLayout;
    
    public AlignToTagCommand(DriveSubsystem drivetrain, PhotonCamera camera, AprilTagFieldLayout fieldLayout) {
        this.drivetrain = drivetrain;
        this.camera = camera;
        this.fieldLayout = fieldLayout;
        addRequirements(drivetrain);

        yawController.setTolerance(30); //Degrees
        rangeController.setTolerance(RANGE_DEADBAND_M);
    }

    @Override
    public void initialize() {
        yawController.reset();
        rangeController.reset();
    }

    @Override
    public void execute() {
        double targetYaw = 0;
        int targetId = -1;
        boolean targetVisible = false;

        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            for (var target : result.getTargets()) {
                for (int id : TARGET_IDS) {
                    if (target.getFiducialId() == id) {
                        targetYaw = target.getYaw();
                        targetId = target.getFiducialId();
                        targetVisible = true;
                        break;
                    }
                }
                if (targetVisible) break;
            }
            
        }

        if (!targetVisible) {
            stop();
            return; 
        }

        //Calculate turn speed from camera yaw
        //Yaw controller drives targetYaw to 0, (centered on tag)
        double turn = yawController.calculate(targetYaw, 0);
        turn = Math.max(-.5, Math.min(.5, turn));

        //Calculate drive speeds from field pose
        double xSpeed = 0;
        double ySpeed = 0;

        var tagPose = fieldLayout.getTagPose(targetId);
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
        SmartDashboard.putNumber("Align/MatchedTagID", targetId);
        SmartDashboard.putBoolean("Align/Target Visible", targetVisible);

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
}
