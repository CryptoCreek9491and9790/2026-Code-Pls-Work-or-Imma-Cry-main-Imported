package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

public class Autos {
    private final AutoFactory factory;
    private final DriveSubsystem drivetrain;
    
    public Autos (DriveSubsystem swerve) {
        this.drivetrain = swerve;
        factory = new AutoFactory(
            swerve::getPose,
            swerve:: resetOdometry,
            swerve::followTrajectory,
            true,
            swerve);
    }



public Command newPath() {
    final var routine = factory.newRoutine("test 2");
    final var traj1 = routine.trajectory("Path1");
    final var traj2 = routine.trajectory("Path2");

    return Commands.sequence(
        Commands.runOnce(routine::poll),
        traj1.resetOdometry(),
        traj1.cmd(),
        Commands.runOnce(()-> drivetrain.drive(0, 0, 0, false), drivetrain),
        Commands.waitSeconds(3),
        traj2.cmd(),
        Commands.runOnce(() -> drivetrain.drive(0, 0, 0, false), drivetrain)
        )
        ;
    }
}
