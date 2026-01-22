// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class RobotContainer {
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

  // The robot's subsystems
  private final DriveSubsystem drivetrain = new DriveSubsystem();

  // The driver's controller
 private final XboxController driverController = new XboxController(0);

 //Camera for vision alignment
 private final PhotonCamera camera = new PhotonCamera("maincam");

 //Vision Alignment Constants
 private static final double VISION_TURN_kP = 0.1;
 private static final double VISION_DES_ANGLE_deg = 0.0;
 private static final double VISION_STRAFE_kP = 0.5;
 private static final double VISION_DES_RANGE_m = 1.5;
 private static final double VISION_RANGE_DEADBAND_m = 0.1; // Stop moving when within 10cm of target
 
 // PID Controller for range control (better than simple P controller)
 private final PIDController rangeController = new PIDController(0.5, 0.0, 0.05);

private int debugCounter = 0;

// Getter method
public XboxController getDriverController() {
    return driverController;}

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    // Configure default commands
    drivetrain.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        // NOTE: Using robot-relative drive (fieldRelative = false) for normal teleop
        new RunCommand(
            () -> {
                double xSpeed = -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband);
                double ySpeed = -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband);
                double rot = -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband);

            if (debugCounter++ % 50 == 0) {
              System.out.println(String.format(
                "Drive inputs - X: %.2f, Y:%.2f, Rot: %.2f (Robot-Relative)", xSpeed, ySpeed, rot));}
                // Robot-relative drive: false = robot-relative, true = field-relative
                drivetrain.drive(xSpeed, ySpeed, rot, false);
            }, drivetrain));}
            
  

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> drivetrain.setX(),
            drivetrain));
  //A Button- Allign to Tag 10
  new JoystickButton(driverController, XboxController.Button.kA.value)
    .whileTrue(new RunCommand(() -> {
      //Get Camera Data
      boolean targetVisible = false;
      double targetYaw = 0;
      double targetRange = 0;

      var results = camera.getLatestResult();
         if (results.hasTargets()) {
          for (var target : results.getTargets()) {
            if (target.getFiducialId() == 10) {
              targetYaw = target.getYaw();
              targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                .5,
                1.435,
                Units.degreesToRadians(-30),
                Units.degreesToRadians(target.getPitch())
                );
              targetVisible = true;
              break;
            }
          }
        }

    //Calculate Alignment Commands
    if (targetVisible) {
      double turn = (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP;
      
      // Calculate distance control using proportional controller
      double rangeError = targetRange - VISION_DES_RANGE_m;
      double forward = 0.0;
      
      // Apply deadband - stop moving forward/backward if within deadband
      if (Math.abs(rangeError) > VISION_RANGE_DEADBAND_m) {
        // When far (targetRange > desired), rangeError is positive, drive forward (positive)
        forward = rangeError * VISION_STRAFE_kP;
      }

      //Clamp Values
      turn = Math.max(-1, Math.min(1, turn));
      forward = Math.max(-1, Math.min(1, forward));

      //Drive Toward Tag
      drivetrain.drive(forward, 0, turn, true);
       // Debug output
                SmartDashboard.putBoolean("Vision Target Visible", true);
                SmartDashboard.putNumber("Target Yaw", targetYaw);
                SmartDashboard.putNumber("Target Range", targetRange);
                SmartDashboard.putNumber("Range Error", targetRange - VISION_DES_RANGE_m);
                SmartDashboard.putNumber("Align Forward", forward);
                SmartDashboard.putNumber("Align Turn", turn);
              } else {
                // No target, stop and reset PID controller
                rangeController.reset();
                drivetrain.drive(0, 0, 0, false);
                SmartDashboard.putBoolean("Vision Target Visible", false);
              }
            },
            drivetrain));
    }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        drivetrain::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drivetrain::setModuleStates,
        drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false));
  }
  public DriveSubsystem getDrivetrain() {
    return drivetrain; // or whatever you named your DriveSubsystem inside RobotContainer
  }
}
