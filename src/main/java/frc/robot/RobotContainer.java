// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.AlignToTagCommand;
import frc.robot.Configs.ShooterSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.Util.FuelSim;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class RobotContainer {
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

  //Dashboard Input
  public FuelSim fuelsim;
  // The robot's subsystems
  private final DriveSubsystem drivetrain = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final Vision vision = new Vision(drivetrain::addVisionMeasurement);

  // The driver's controller
 private final XboxController driverController = new XboxController(0);
 private final XboxController opController = new XboxController(1);

 //Camera for vision alignment
 private final PhotonCamera camera = new PhotonCamera("maincam");
 
 // AprilTag field layout for getting tag poses
 private final AprilTagFieldLayout fieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

 //Vision Alignment Constants
 private static final double VISION_TURN_kP = 0.1;
 private static final double VISION_STRAFE_kP = 0.5;
 private static final double VISION_DES_RANGE_m = 1.5;
 private static final double VISION_RANGE_DEADBAND_m = 0.1; // Stop moving when within 10cm of target
 
 // PID Controller for range control (better than simple P controller)
 private final PIDController rangeController = new PIDController(0.5, 0.0, 0.05);

private int debugCounter = 0;

//Auto stuff
private final Autos autos = new Autos(drivetrain);
private final SendableChooser<Command> autoChooser = new SendableChooser<>(); 

// Getter method
public XboxController getDriverController() {
    return driverController;}

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings(
  
    );
    
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
            }, drivetrain));
      autoChooser.setDefaultOption("test 2", autos.newPath());
    SmartDashboard.putData("autoChooser", autoChooser);

    if (RobotBase.isSimulation()) {
      configureFuelSim();}}
    
  
            
  private void configureFuelSim(){
  }
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
    .onTrue(intakeSubsystem.runUpCommand());

    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
    .onTrue(intakeSubsystem.runDownCommand());

    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .whileTrue(new RunCommand(
            () -> drivetrain.setX(),
            drivetrain));

    new JoystickButton(driverController, XboxController.Button.kA.value)
      .whileTrue(intakeSubsystem.runIntakeCommand());

  //A Button- Allign to Tag 25
  new JoystickButton(driverController, XboxController.Button.kA.value)
    .whileTrue(new AlignToTagCommand(drivetrain, vision.getFrontLeftCamera(), vision.getFrontRightCamera(), fieldLayout));
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public DriveSubsystem getDrivetrain() {
    return drivetrain;
  }

  public Vision getVision() {
    return vision;
  }
}
