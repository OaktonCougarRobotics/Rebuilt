package frc.robot.commands;

import java.util.HashSet;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends Command{
     
    private Drivetrain drivetrain;
    private DoubleSupplier xTranslationSupplier;
    private DoubleSupplier yTranslationSupplier;
    private DoubleSupplier thetaTranslationSupplier;

/**
     * Constructs a DriveCommand command
     * 
     * @param swerveDrive the swerveDrive instance
     * @param xDoubleSupplier {@link java.util.function.DoubleSupplier DoubleSupplier} that supplies the double value between [-1,1] for joystick x translation
     * @param yDoubleSupplier {@link java.util.function.DoubleSupplier DoubleSupplier} that supplies the double value between [-1,1] for joystick y translation
     * @param thetaDoubbleSupplier {@link java.util.function.DoubleSupplier DoubleSupplier} that supplies the double value between [-1,1] for joystick theta translation
     */
    public DriveCommand(Drivetrain drivetrain, DoubleSupplier xTranslationSupplier, DoubleSupplier yTranslationSupplier, DoubleSupplier thetaTranslationSupplier){
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
        drivetrain.swerveDrive.driveFieldOriented(new ChassisSpeeds(

            deadzone(xTranslationSupplier.getAsDouble(),0.05)
              * drivetrain.swerveDrive.getMaximumChassisVelocity(),

            deadzone(yTranslationSupplier.getAsDouble(),0.05)
              * drivetrain.swerveDrive.getMaximumChassisVelocity(),

            deadzone(thetaTranslationSupplier.getAsDouble(),0.05)
              * drivetrain.swerveDrive.getMaximumChassisAngularVelocity()),
          new Translation2d());
    }
    @Override
    public void end(boolean interrupted){
        
    } 

    public boolean isFinished(){
        
        return false;
    }

    @Override
    public boolean runsWhenDisabled(){
        return false;
    }

    public HashSet<Subsystem> getRequirements(){
        HashSet<Subsystem> req = new HashSet<>();
        req.add(drivetrain);
        return req;
    }

    public static double deadzone(double num, double deadband){
        if (Math.abs(num) < deadband)
          return 0.0;
        return num;
      }
}
