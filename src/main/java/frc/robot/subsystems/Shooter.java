package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;

public class Shooter {
    private TalonFX shooterMotor;
    private PIDController speedController;

    public Shooter(int shooterMotorID) {
        shooterMotor = new TalonFX(shooterMotorID);
        shooterMotor.setPosition(0);
        speedController = new PIDController(1, 0, 0);
    }
}
