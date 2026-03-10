package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.RobotState;

public class Intake extends SubsystemBase {
    public TalonFX intakeWheel;
    public TalonFX feederWheel;
    public Supplier<RobotState> robotStateSupplier;
    public Intake(int intakeWheelId,
        int feederWheelId,
        Supplier<RobotState> robotStateSupplier, 
        PIDController intakeController, 
        PIDController feederController
    ) {
        this.intakeWheel = new TalonFX(intakeWheelId);
        this.feederWheel = new TalonFX(feederWheelId);
        this.robotStateSupplier = robotStateSupplier;
    }
    @Override
    public void periodic(){
        // if(robotStateSupplier.get()==RobotState.INTAKE) {
        //     intakeWheel
        // }
        // else if() {

        // }
        // else{
        //     //neutral stuff
        // }
    }
}
