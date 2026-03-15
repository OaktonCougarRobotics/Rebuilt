package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    Shooter shooter;
    public Supplier<Boolean> isShooting;

    public Supplier<Boolean> isManual;
    public Supplier<Double> manualOutput;
    public Supplier<Boolean> manualIndex;
    public ShootCommand(
        Shooter shooter,
        Supplier<Boolean> isShooting,
        Supplier<Boolean> isManual,
        Supplier<Double> manualOutput,
        Supplier<Boolean> manualIndex

    ){
        this.shooter = shooter;
        this.isManual = isManual;
        this.isShooting = isShooting;
        this.manualOutput = manualOutput;
        this.manualIndex = manualIndex;

    }
    public Set<Subsystem> getRequirements(){
        Set<Subsystem> x = new HashSet<Subsystem>();
        x.add(shooter);
        return x;        
    }
    @Override
    public void execute(){
        if(isManual.get()){
            System.out.println("SHOOTER IS MANUAL");
            double a = manualOutput.get();
            shooter.shooterMotor.setVoltage(Constants.MAX_FLYWHEEL_VOLTAGE * a); //make sure button has range [-1,1]
            shooter.indexMotor.set(manualIndex.get()?1:-1);
        }
        else if (isShooting.get()){ //not manual and shooting rn
            shooter.shooterMotor.setVoltage(10); // not manual and not shooting
            // shooter.indexMotor.setVoltage(8);
            shooter.indexMotor.setControl(new VoltageOut(12));
        }
        else {
            shooter.shooterMotor.setVoltage(0); // not manual and not shooting
            shooter.indexMotor.setVoltage(0);
        }
    }

    @Override
    public void end(boolean interrupted){
        shooter.shooterMotor.setVoltage(0);
        shooter.indexMotor.setVoltage(0);
        // shooter.indexMotor.setControl(new VoltageOut(0));
    } 
    // private PIDController shootController;
    // private PIDController hoodController;

    // private DoubleSupplier flyWheelSpeed;
    // private DoubleSupplier hoodPosition;

    // private final double flyWheel_kP = 0;
    // private final double flyWheel_kI = 0;
    // private final double flyWheel_kD = 0;

    // private final double hoodPosition_kP = 0;
    // private final double hoodPosition_kI = 0;
    // private final double hoodPosition_kD = 0;

    // private Shooter shooter;

    // public ShootCommand(Shooter shooter, DoubleSupplier flyWheelSpeed, DoubleSupplier hoodPosition) {
    //     this.shooter = shooter;
    //     shootController = new PIDController(flyWheel_kP, flyWheel_kI, flyWheel_kD);
    //     hoodController = new PIDController(hoodPosition_kP, hoodPosition_kI, hoodPosition_kD);

    //     this.flyWheelSpeed = flyWheelSpeed;
    //     this.hoodPosition = hoodPosition;
    // }

    // @Override
    // public void initialize() {

    // }

    // @Override
    // public void execute() {
    //     shooter.getShooterMotor().setControl(new DutyCycleOut(shootController.calculate(shooter.getShooterMotor().getPosition().getValueAsDouble(), flyWheelSpeed.getAsDouble())));
    //     // shooter.gethoodMotor().setPosition(hoodController.calculate(shooter.gethoodMotor().getPosition().getValueAsDouble(), hoodPosition.getAsDouble()));
    // }

    // @Override
    // public void end(boolean interrupted){
        
    // } 

    // @Override
    // public boolean runsWhenDisabled(){
    //     return false;
    // }
}
