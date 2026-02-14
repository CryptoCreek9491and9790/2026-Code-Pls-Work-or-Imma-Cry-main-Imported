package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.Constants.IntakeSubsystemConstants.PivotSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;

public class IntakeSubsystem extends SubsystemBase {
    //Initialize intake SPARK. We will use open loop control for this
    private final SparkMax IntakeMotor =
    new SparkMax(IntakeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
    
    //Initialize intake SPARK. We will use open loop control for this
    private final SparkMax PivotMotor =
        new SparkMax(IntakeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushless);

    public IntakeSubsystem() {

        IntakeMotor.configure(
        Configs.IntakeSubsystem.INTAKE_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        PivotMotor.configure(
        Configs.IntakeSubsystem.PIVOT_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        System.out.println("IntakeSubsystem Initialized");
    }

    //Set the intake motor power in the range of [-1, 1]
    private void setIntakePower(double power) {
        IntakeMotor.set(power);
    }

    //Set the intake pivot power in the range of [-1,1]
    private void setPivotPower(double power) {
        PivotMotor.set(power);
    }

    //Command to run the intake and pivot motors. When the command is interrupted,
    // ex is if the button is released, the motors will stop


    public Command runDownCommand() {
        return this.startEnd(()-> {
        this.setPivotPower(PivotSetpoints.kDown); },
        () -> {
            this.setPivotPower(.1);
            }).withName("Going Down");
        }

    //Command to reverse the intake motor and pivot motor. When the command is interrupted, 
    //ex the buttons is released, the motors will stop.
    public Command runUpCommand() {
        return this.startEnd( () -> {
        this.setPivotPower(PivotSetpoints.kUp); },
        () -> {
            this.setPivotPower(.1);
        }).withName("Going Up");
    }}
