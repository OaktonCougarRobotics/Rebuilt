package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.RobotState;

public class Outtake extends SubsystemBase{
    public TalonFX indexMotor;
    public TalonFX shooterMotor;
    public Supplier<RobotState> robotStateSupplier;
    public Supplier<Boolean> isShooting;
    public Outtake(int indexMotorID,
        int shooterMotorID,
        Supplier<Boolean> shoot) {
            this.indexMotor = new TalonFX(indexMotorID);
            this.shooterMotor = new TalonFX(shooterMotorID);
            this.isShooting = shoot;
    }
    public void periodic(){
        if(isShooting.get()){indexMotor.setVoltage(6); shooterMotor.setVoltage(7);}
        else {indexMotor.setVoltage(0); shooterMotor.setVoltage(0);}
    }

    // public Command runIndex(double i) {
    //     return Commands.runOnce(() -> indexMotor.setVoltage(i));
    // }

    // public Command runShooter(double i) {
    //     return Commands.runOnce(() -> shooterMotor.setVoltage(i));
    // }
}
