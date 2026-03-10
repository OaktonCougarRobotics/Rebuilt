package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    private PIDController shootController;
    private PIDController hoodController;

    private DoubleSupplier flyWheelSpeed;
    private DoubleSupplier hoodPosition;

    private final double flyWheel_kP = 0;
    private final double flyWheel_kI = 0;
    private final double flyWheel_kD = 0;

    private final double hoodPosition_kP = 0;
    private final double hoodPosition_kI = 0;
    private final double hoodPosition_kD = 0;

    private Shooter shooter;

    public ShootCommand(Shooter shooter, DoubleSupplier flyWheelSpeed, DoubleSupplier hoodPosition) {
        this.shooter = shooter;
        shootController = new PIDController(flyWheel_kP, flyWheel_kI, flyWheel_kD);
        hoodController = new PIDController(hoodPosition_kP, hoodPosition_kI, hoodPosition_kD);

        this.flyWheelSpeed = flyWheelSpeed;
        this.hoodPosition = hoodPosition;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.getShooterMotor().setControl(new DutyCycleOut(shootController.calculate(shooter.getShooterMotor().getPosition().getValueAsDouble(), flyWheelSpeed.getAsDouble())));
        // shooter.gethoodMotor().setPosition(hoodController.calculate(shooter.gethoodMotor().getPosition().getValueAsDouble(), hoodPosition.getAsDouble()));
    }

    @Override
    public void end(boolean interrupted){
        
    } 

    @Override
    public boolean runsWhenDisabled(){
        return false;
    }
}
