package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;

public class Shooter {
    private TalonFX shooterMotor;
    private TalonFX hoodMotor;

    public Shooter(int shooterMotorID, int hoodMotorID) {
        shooterMotor = new TalonFX(shooterMotorID);
        shooterMotor.setPosition(0);

        hoodMotor = new TalonFX(hoodMotorID);
        hoodMotor.setPosition(0);
    }

    public TalonFX getShooterMotor() {
        return shooterMotor;
    }

    public TalonFX gethoodMotor() {
        return hoodMotor;
    }
}