package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants;

public class ShooterSubsystem extends SubsystemBase {
   
    private final SparkMax ShooterMotor =
    new SparkMax(ShooterSubsystemConstants.kShooterMotorCanId, MotorType.kBrushless);

    private final SparkMax BackRollerMotor = 
        new SparkMax(ShooterSubsystemConstants.KBackRollerMotorCanId, MotorType.kBrushless);

    private final SparkClosedLoopController shooterController = ShooterMotor.getClosedLoopController();
    private final SparkClosedLoopController backrollerController = BackRollerMotor.getClosedLoopController();

    private final RelativeEncoder shooterEncoder = ShooterMotor.getEncoder();
    private final RelativeEncoder backrollerEncoder = BackRollerMotor.getEncoder();

    private static final InterpolatingDoubleTreeMap shooterRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap backrollerRPMMap = new InterpolatingDoubleTreeMap();


    static {
        //Distance(meters) -> shooter power
        //Vortex Free Speed is ~6784 RPM, so dont exceed this
        //Both wheels equal at close range, diff increases at distance
        //to add more arc
        //Change later
        shooterRPMMap.put(1.0, 2500.0);
        shooterRPMMap.put(2.0, 3200.0);
        shooterRPMMap.put(3.0, 3900.0);
        shooterRPMMap.put(4.0, 4600.0);
        shooterRPMMap.put(5.0, 5300.0);

        //Distance(meters) -> backroller power
        //Tune the ratio between shooter and backroller to shape the shot arc
        //Change later
        backrollerRPMMap.put(1.0, 2500.0); //eqaul at close range = flatter arc
        backrollerRPMMap.put(2.0, 2600.0); 
        backrollerRPMMap.put(3.0, 2800.0);
        backrollerRPMMap.put(4.0, 3000.0);
        backrollerRPMMap.put(5.0, 3200.0);   
    }
    

    public ShooterSubsystem() {
        ShooterMotor.configure(
        Configs.ShooterSubsystem.SHOOTER_CONFIG,
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters);

        BackRollerMotor.configure(
            Configs.ShooterSubsystem.BACKROLLER_CONFIG,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    private void setBackRollerRPM(double rpm) {
        backrollerController.setSetpoint(rpm , ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    private void setShooterRPM(double rpm) {
        shooterController.setSetpoint(rpm , ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    private void stopAll() {
        ShooterMotor.set(0);
        BackRollerMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Flywheel RPM", shooterEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/Backroller RPM", backrollerEncoder.getVelocity());
    }

    public Command shootCommand(DoubleSupplier distanceMeters) {
        return this.run(() -> {
            double dist = distanceMeters.getAsDouble();

            //Clamp distance to table bouds so it doesnt extrapolate wildly
            dist = Math.max(1.0, Math.min(5.0, dist));

            //double shooterRPM = shooterRPMMap.get(dist);
            double backrollerRPM = backrollerRPMMap.get(dist);

            //setShooterRPM(shooterRPM);
            setBackRollerRPM(backrollerRPM);

            SmartDashboard.putNumber("Shooter/Distance (m)", dist);
            //SmartDashboard.putNumber("Shooter/Flywheel RPM", shooterRPM);
            SmartDashboard.putNumber("Shooter/Backroller RPM", backrollerRPM);
        }).finallyDo(interrupted -> stopAll()).withName("Shoot");
    }
    
    //Fallback Command for when vision is not available
    //Runs both wheels at a fixed flat power
    public Command shootFixedCommand() {
        return this.startEnd(
            () -> {
                setShooterRPM(4000);
                setBackRollerRPM(5000);
            }, this::stopAll)
            .withName("Shoot Fixed");
    }
}

