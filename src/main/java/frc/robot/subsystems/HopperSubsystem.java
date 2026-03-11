package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.HopperSubsystemConstants;
import frc.robot.Constants.HopperSubsystemConstants.IndexerSetpoints;
import frc.robot.Constants.HopperSubsystemConstants.RollerSetpoints;

public class HopperSubsystem extends SubsystemBase {
    
    private final SparkMax rollerMotor =
        new SparkMax(HopperSubsystemConstants.krollerMotorCanId,MotorType.kBrushless);

    private final SparkMax indexerMotor =
        new SparkMax(HopperSubsystemConstants.kindexerMotorCandId, MotorType.kBrushless);
    
    
    public HopperSubsystem() {
      rollerMotor.configure(Configs.HopperSubsystem.ROLLER_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
      
      indexerMotor.configure(Configs.HopperSubsystem.INDEXER_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    } 

    private void setRollerPower(double power) {
        rollerMotor.set(power);
    }

    private void setindexerPower(double power) {
        indexerMotor.set(power);
    }

    public Command rollCommand() {
        return this.startEnd(
            () -> {
                this.setRollerPower(RollerSetpoints.kRoll);
                this.setindexerPower(IndexerSetpoints.kIndex);
            }, () -> {
                this.setRollerPower(0);
                this.setindexerPower(0);
            } );
    }
}
