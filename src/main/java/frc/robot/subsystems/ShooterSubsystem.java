package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.Constants.ShooterSubsystemConstants.HoodSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants.ShooterSetpoints;

public class ShooterSubsystem extends SubsystemBase {
   
    private final SparkMax ShooterMotor =
    new SparkMax(ShooterSubsystemConstants.kShooterMotorCanId, MotorType.kBrushless);

    private final SparkMax HoodMotor = 
        new SparkMax(ShooterSubsystemConstants.KHoodMotorCanId, MotorType.kBrushless);
    private final SparkClosedLoopController HoodController = HoodMotor.getClosedLoopController();
    private final AbsoluteEncoder HoodEncoder = HoodMotor.getAbsoluteEncoder();

    public ShooterSubsystem() {
        ShooterMotor.configure(
        Configs.ShooterSubsystem.SHOOTER_CONFIG,
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters);

        HoodMotor.configure(
            Configs.ShooterSubsystem.HOOD_CONFIG,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    private void setHoodAngle(double degrees) {
        HoodController.setSetpoint(degrees, ControlType.kPosition);
    }
    private void setShooterPower (double power) {
        ShooterMotor.set(power);
    }


    public double getHoodAngle() {
        return HoodEncoder.getPosition();
    }

    public boolean HoodAtSetpoint(double targetDegrees) {
        return Math.abs(getHoodAngle() - targetDegrees) < ShooterSubsystemConstants.kHoodToleranceDegrees;
    }
    public Command shootCommand() {
        return this.startEnd(
            () -> this.setShooterPower(ShooterSetpoints.kShoot),
            () -> this.setShooterPower(ShooterSetpoints.kidle)
        ).withName("Shoot");
    }
    
    public Command hoodCloseCommand () {
        return this.runOnce(
            () -> setHoodAngle(HoodSetpoints.kClose))
                .withName("Hood Close Shot");
    }

    public Command hoodFarCommand() {
        return this.runOnce(
            () -> setHoodAngle(HoodSetpoints.kFar))
            .withName("Hood Far Shot");
    }

    public Command hoodStowCommand() {
        return this.runOnce(() -> setHoodAngle(HoodSetpoints.kStow))
        .withName("Hood is Stowed");
    }
}
