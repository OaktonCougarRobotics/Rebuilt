package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class RotationDifferenceToHub extends Command{
    
    private Drivetrain drivetrain;
    private DoubleSupplier xTranslationSupplier;
    private DoubleSupplier yTranslationSupplier;
    private DoubleSupplier thetaTranslationSupplier;

    public RotationDifferenceToHub(Drivetrain drivetrain, DoubleSupplier xTranslationSupplier, DoubleSupplier yTranslationSupplier, DoubleSupplier thetaTranslationSupplier){
        this.drivetrain = drivetrain;
        this.xTranslationSupplier = xTranslationSupplier;
        this.yTranslationSupplier = yTranslationSupplier;
        this.thetaTranslationSupplier = thetaTranslationSupplier;
    }



    @Override
    public void initialize() {

    }
    @Override
    public void execute(){
        
    }
    @Override
    public void end(boolean interrupted){
        
    } 
    public double pid(double measurement, double setpoint){
        PIDController pid = new PIDController(0, 0, 0);

        double motorControl = pid.calculate(measurement, setpoint);

        pid.close();
        return motorControl;
    }

}
