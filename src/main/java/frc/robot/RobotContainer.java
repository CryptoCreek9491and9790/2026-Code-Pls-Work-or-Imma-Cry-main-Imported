// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.AlignToTagCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Util.FuelSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import choreo.auto.AutoChooser;
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
  private final HopperSubsystem hopperSubsystem = new HopperSubsystem();

  // The driver's controller
 private final XboxController driverController = new XboxController(0);
 private final XboxController opController = new XboxController(1);

  
 // AprilTag field layout for getting tag poses
 private final AprilTagFieldLayout fieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

 //Field Relativity
 private boolean fieldRelative = true;

 //Auto stuff
//private final Autos autos = new Autos(drivetrain, shooterSubsystem, hopperSubsystem, intakeSubsystem, vision);
//AutoChooser.setDefaultOption("Full Auto", autos.fullAuto());
//AutoChooser.addOption("Simple Shoot", autos.simpleShootAuto());
//SmartDashboard.putData("autoChooser", autoChooser);

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

                // Robot-relative drive: false = robot-relative, true = field-relative
                SmartDashboard.putString("Drive Mode", fieldRelative ? "Field-Relative" : "Robot-Relative");


                drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);
            }, drivetrain));
      //autoChooser.setDefaultOption("test 2", autos.newPath());
 //   SmartDashboard.putData("autoChooser", autoChooser);

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


    new JoystickButton(driverController, XboxController.Button.kBack.value)
      .onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));

    new JoystickButton(driverController, XboxController.Button.kY.value)
      .onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

      //new JoystickButton(driverController, XboxController.Button.kY.value)
      //.toggleOnTrue(shooterSubsystem.hoodStowCommand());

    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
    .onTrue(intakeSubsystem.runUpCommand())
    .onFalse(intakeSubsystem.runStopCommand());

    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
    .onTrue(intakeSubsystem.runDownCommand())
    .onFalse(intakeSubsystem.runStopCommand());

    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .whileTrue(new RunCommand(
            () -> drivetrain.setX(),
            drivetrain));

    new JoystickButton(opController, XboxController.Button.kY.value)
      .toggleOnTrue(intakeSubsystem.runIntakeCommand());

    new JoystickButton(opController, XboxController.Button.kX.value)
      .toggleOnTrue(hopperSubsystem.rollCommand());

    new JoystickButton(opController, XboxController.Button.kB.value)
      .toggleOnTrue(shooterSubsystem.shootCommand(() -> vision.getHubDistance()));

    new JoystickButton(opController, XboxController.Button.kA.value)
      .toggleOnTrue(shooterSubsystem.shootFixedCommand());

  //A Button- Allign to Tag 25
  new JoystickButton(driverController, XboxController.Button.kB.value)
    .whileTrue(new AlignToTagCommand(drivetrain, vision.getFrontLeftCamera(), vision.getFrontRightCamera(), fieldLayout));
  }
  

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    //return autoChooser.getSelected();
  //}

  public DriveSubsystem getDrivetrain() {
    return drivetrain;
  }

  public Vision getVision() {
    return vision;
  }
}
