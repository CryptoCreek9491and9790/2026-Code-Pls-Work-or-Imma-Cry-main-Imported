// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import com.studica.frc.AHRS;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  public SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveOdometry m_odometry;

  //Sim
  private double m_simGyroAngle = 0.0; //Sim gyro angle
  private ChassisSpeeds m_lastChassisSpeeds = new ChassisSpeeds(); //Track last commanded speeds

  //PID for Auto
  private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(10.0, 0, 0);
  private final PIDController headingController = new PIDController(7.5, 0, 0);

  /** Creates a new DriveSubsystem. */
  
  public DriveSubsystem() {

    //Heading for Auto
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
   // Define the standard deviations for the pose estimator, which determine how fast the pose
        // estimate converges to the vision measurement. This should depend on the vision measurement
        // noise
        // and how many or how frequently vision measurements are applied to the pose estimator.
        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
        poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(m_gyro.getYaw()), new SwerveModulePosition[] {m_frontLeft.getPosition(),
           m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
             m_rearRight.getPosition()},
              new Pose2d(),
               stateStdDevs, visionStdDevs);

    m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }
      );
    }

  @Override
  public void periodic() {
      // Create the positions array
      SwerveModulePosition[] positions = new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      };
      
      // Get the gyro rotation
      Rotation2d rotation = Rotation2d.fromDegrees(getGyroAngle());
      
      // Update both odometry and pose estimator with the same values
      m_odometry.update(rotation, positions);
      poseEstimator.update(rotation, positions);
  }

  @Override
  public void simulationPeriodic() {
    m_frontLeft.simulationPeriodic();
    m_frontRight.simulationPeriodic();
    m_rearLeft.simulationPeriodic();
    m_rearRight.simulationPeriodic();

    double omegaRadPerSec = m_lastChassisSpeeds.omegaRadiansPerSecond;
    double gyroScale = .007;
    
    m_simGyroAngle += Math.toDegrees(omegaRadPerSec * gyroScale) * .2;

    while (m_simGyroAngle > 180) m_simGyroAngle -= 360;
    while (m_simGyroAngle < -180) m_simGyroAngle += 360;
  }

  private double getGyroAngle() {
    if (RobotBase.isSimulation()) {
      return m_simGyroAngle;
    } else {
      return m_gyro.getAngle();

    }
    }
  
  public void log() {
        // Robot pose (position on the field, if odometry is set up)
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Robot X (m)", getPose().getX());
        SmartDashboard.putNumber("Robot Y (m)", getPose().getY());
        SmartDashboard.putNumber("Robot Heading (deg)", getHeading());

        //Add pose array for field visualization
        SmartDashboard.putNumberArray("Robot Pose", new double[] {
          pose.getX(),
          pose.getY(),
          pose.getRotation().getRadians()});
          
        SmartDashboard.putNumberArray("Swerve States", new double[] {
          m_frontLeft.getState().angle.getRadians(),
          m_frontLeft.getState().speedMetersPerSecond,
          m_frontRight.getState().angle.getRadians(),
          m_frontRight.getState().speedMetersPerSecond,
          m_rearLeft.getState().angle.getRadians(),
          m_rearLeft.getState().speedMetersPerSecond,
          m_rearRight.getState().angle.getRadians(),
          m_rearRight.getState().speedMetersPerSecond});

        // Example module logging (if you want individual wheel states)
        // Adjust names as needed
        Pose2d odomPose = m_odometry.getPoseMeters();
        SmartDashboard.putNumber("Front Left Module Angle (deg)", m_frontLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("Front Left Module Speed (m/s)", m_frontLeft.getState().speedMetersPerSecond);

        SmartDashboard.putNumber("Front Right Module Angle (deg)", m_frontRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("Front Right Module Speed (m/s)", m_frontRight.getState().speedMetersPerSecond);

        SmartDashboard.putNumber("Back Left Module Angle (deg)", m_rearLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("Back Left Module Speed (m/s)", m_rearLeft.getState().speedMetersPerSecond);

        SmartDashboard.putNumber("Back Right Module Angle (deg)", m_rearRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("Back Right Module Speed (m/s)", m_rearRight.getState().speedMetersPerSecond);

        //Gyro Info
        SmartDashboard.putNumber("Gyro Yaw (deg)", getGyroAngle());
        SmartDashboard.putNumber("Gyro Pitch", m_gyro.getPitch());
        SmartDashboard.putNumber("Gyro Roll", m_gyro.getRoll());
        
        //Drive mode info
        SmartDashboard.putString("Drive Mode", "Robot-Relative");
    }

    /** Returns heading from the gyro in degrees. */
        public double getHeading() {
        return Rotation2d.fromDegrees(getGyroAngle()).getDegrees();
      }
      


  /**
   * Returns the currently-estimated pose of the robot.
   * Uses pose estimator which fuses odometry and vision.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }


  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  
  public void resetOdometry(Pose2d pose) {
    SwerveModulePosition[] positions = new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
    Rotation2d rotation =Rotation2d.fromDegrees(m_gyro.getAngle());

    m_odometry.resetPosition(rotation, positions, pose);
    poseEstimator.resetPosition(rotation, positions, pose);

    if (RobotBase.isSimulation()) {
      m_simGyroAngle = pose.getRotation().getDegrees();
    }
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionMeasurement The vision measurement pose.
   * @param timestampSeconds  The timestamp of the vision measurement in seconds.
   */

  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
}

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    // Significantly increase turning speed in simulation for better responsiveness
    // Apply multiplier to the input before scaling to max speed
    double rotInput = rot;
    if (RobotBase.isSimulation()) {
      rotInput = rot * 10.0; // 10x multiplier for simulation
    }
    double rotDelivered = rotInput * DriveConstants.kMaxAngularSpeed;

    // Robot-relative: speeds are relative to robot's current orientation
    // Field-relative: speeds are relative to field (requires gyro)
    if (fieldRelative) {
      // Field-relative drive: transform speeds based on robot's heading
      m_lastChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeedDelivered,
        ySpeedDelivered,
        rotDelivered,
        Rotation2d.fromDegrees(getGyroAngle()));
    } else {
      // Robot-relative drive: speeds are directly applied (no gyro transformation)
      m_lastChassisSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    }

  if (Math.abs(xSpeedDelivered) > .1 || Math.abs(ySpeedDelivered) > .1 || Math.abs(rotDelivered) > .1) {
    String simStatus = RobotBase.isSimulation() ? " (SIM - 10x turn)" : "";
    System.out.println(String.format(
      "Driving - X: %.2f m/s, Y: %.2f m/s, Rot: %.2f rad/s%s", xSpeedDelivered, ySpeedDelivered, rotDelivered, simStatus));}

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        m_lastChassisSpeeds);
    
        SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);  

  System.out.println(String.format(
    "FL:  %.2f m/s @ %.1f°, FR: %.2f m/s @ %.1f",
    swerveModuleStates[0].speedMetersPerSecond, swerveModuleStates[0].angle.getDegrees(),
    swerveModuleStates[1].speedMetersPerSecond, swerveModuleStates[1].angle.getDegrees()
  ));

  m_frontLeft.setDesiredState(swerveModuleStates[0]);
  m_frontRight.setDesiredState(swerveModuleStates[1]);
  m_rearLeft.setDesiredState(swerveModuleStates[2]);
  m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    if (RobotBase.isSimulation()) {
      m_simGyroAngle = 0;
    } else {
    m_gyro.reset();}
  }

  public void followTrajectory(SwerveSample sample) {
    //Logging pose for Advantage Scope
    SmartDashboard.putNumberArray("Choreo Target Pose", new double[] {
      sample.x,
      sample.y,
      sample.heading
    });

    Pose2d pose = getPose();

    ChassisSpeeds speeds = new ChassisSpeeds(
      sample.vx + xController.calculate(pose.getX(), sample.x),
      sample.vy +yController.calculate(pose.getY(), sample.y),
      sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
    );

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation()));
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
