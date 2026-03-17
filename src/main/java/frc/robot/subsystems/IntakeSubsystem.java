package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;

public class IntakeSubsystem extends SubsystemBase {
    //Initialize intake SPARK. We will use open loop control for this
    private final SparkMax IntakeMotor =
    new SparkMax(IntakeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
    
    //Initialize intake SPARK. We will use open loop control for this
    private final SparkMax PivotMotor =
        new SparkMax(IntakeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushless);
    private final SparkClosedLoopController pivotController = PivotMotor.getClosedLoopController();
    private final AbsoluteEncoder pivotEncoder = PivotMotor.getAbsoluteEncoder();


    public IntakeSubsystem() {

        IntakeMotor.configure(
        Configs.IntakeSubsystem.INTAKE_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        PivotMotor.configure(
        Configs.IntakeSubsystem.PIVOT_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    }

    

    //Set the intake motor power in the range of [-1, 1]
    private void setIntakePower(double power) {
        IntakeMotor.set(power);
    }

    private void setPivotAngle(double degrees) {
        pivotController.setSetpoint(degrees, ControlType.kPosition);
    }



    public double getPivotAngle() {
        return pivotEncoder.getPosition();
    }

    private void setPivotPower(double power) {
        PivotMotor.set(power);
    }

    

    public boolean pivotAtSetpoint( double targetDegrees) {
        return Math.abs(getPivotAngle() - targetDegrees) < IntakeSubsystemConstants.kPivotToleranceDegrees;
    }

    //Command to run the intake and pivot motors. When the command is interrupted,
    // ex is if the button is released, the motors will stop


    public Command runDownCommand() {
        return new InstantCommand(() -> {
            setPivotPower(.2);
           }
            ,this);
        }




    public Command runUpCommand() {
        return new InstantCommand(() ->{
            setPivotPower(-.2);
        }
        ,this);
    }

    public Command runStopCommand() {
        return new InstantCommand( () -> {
            setPivotPower(0);
        }
        ,this);
    }

    public Command runIntakeCommand() {
        return this.startEnd( () ->
        setIntakePower(-.4),
        () -> setIntakePower(0));
    }
    //Reverses the intake roller to eject balls
    //Pivot stays where it is
    public Command ejectCommand() {
        return this.startEnd(() -> setIntakePower(IntakeSetpoints.kExtake),
         () -> setIntakePower(0)
        ).withName("Eject");
    }
}
