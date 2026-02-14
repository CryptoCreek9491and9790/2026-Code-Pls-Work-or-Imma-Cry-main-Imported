package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

public class Autos {
    private final AutoFactory factory;
    
    public Autos (DriveSubsystem swerve) {
        factory = new AutoFactory(
            swerve::getPose,
            swerve:: resetOdometry,
            swerve::followTrajectory,
            true,
            swerve);
    }



public Command newPath() {
    final var routine = factory.newRoutine("test 2");
    final var traj = routine.trajectory("NewPath");

    routine.active().whileTrue(Commands.sequence(
        traj.resetOdometry(),
        traj.cmd()
        )
        );

        return routine.cmd();}
}
