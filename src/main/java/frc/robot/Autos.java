package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Configs.IntakeSubsystem;
import frc.robot.Configs.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class Autos {
    /*private final AutoFactory factory;
    private final DriveSubsystem drivetrain;
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final IntakeSubsystem intake;

    private static final double SHOOTER_SPINUP_SECONDS = 1;

    private static final double SHOOT_DURATION_SECONDS = 2.5;

    private static final double INTAKE_DURATION_SECONDS = 2;
    
    public Autos (DriveSubsystem drivetrain,
                  ShooterSubsystem shooter,
                  HopperSubsystem hopper,
                  IntakeSubsystem intake) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.hopper = hopper;
        this.intake = intake;

        factory = new AutoFactory(
            drivetrain::getPose,
            drivetrain:: resetOdometry,
            drivetrain::followTrajectory,
            true,
            drivetrain);
    }

 
   1.Drive Back towards shooting zone
    2.Spin up shooters then shoot preload
    3.Drive to depot station and collect
    4.Go back to shoot zone
    5. spin up shooter then shoot
    6. Drive to hang
    7. Hang

  

  public Command fullAuto() {
    var routine = factory.newRoutine("Full Auto");

    var driveToHub = routine.trajectory("DriveToHub");
    var hubToDepot = routine.trajectory("DepotToHub");
    var DepotToHub = routine.trajectory("DepotToHub");
    var hubToHang = routine.trajectory("HubToHang");

    routine.active().onTrue(
        Commands.sequence(
            //Reset odom and drive to hub
            driveToHub.resetOdometry(),
            Commands.parallel(
                driveToHub.cmd(),
            intake.runDownCommand() //deploy intake arm while driving
        ),
        //Spin up shooter and set hood
        //Run shooter motor and set hood simultaneously, then wait for spinup
        Commands.parallel(
            shooter.hoodCloseCommand(),
            shooter.shootCommand().withTimeout(SHOOTER_SPINUP_SECONDS)
        ),

        //Feed hopper to shoot preloaded fuel
        //Keep shooter running and run hopper to feed balls in
        Commands.parallel(
            shooter.shootCommand().withTimeout(SHOOT_DURATION_SECONDS),
            hopper.rollCommand().withTimeout(SHOOT_DURATION_SECONDS)
        ),

        Commands.runOnce(() -> drivetrain.drive(0, 0, 0, true)), drivetrain));


    
  }
public Command newPath() {
    var routine = factory.newRoutine("test 2");
    var traj1 = routine.trajectory("Path1");
    var traj2 = routine.trajectory("Path2");

    routine.active().onTrue(
        Commands.sequence(
        traj1.resetOdometry(),
        traj1.cmd(),
        Commands.runOnce(()-> drivetrain.drive(0, 0, 0, false), drivetrain),
        Commands.waitSeconds(3),
        traj2.cmd(),
        Commands.runOnce(() -> drivetrain.drive(0, 0, 0, false), drivetrain)
        )
    );    
    return routine.cmd();
}*/ }
