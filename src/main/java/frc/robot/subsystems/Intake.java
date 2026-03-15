package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer.RobotState;

public class Intake extends SubsystemBase {
    public TalonFX intakeMotor;
    public TalonFX feederWheel;
    public Supplier<RobotState> robotStateSupplier;

    // public boolean feederManual = false;
    // public boolean intakeManual = false;
    public Intake(
        int intakeWheelId,
        int feederWheelId,
        Supplier<RobotState> robotStateSupplier
    ) {
        this.intakeMotor = new TalonFX(intakeWheelId);
        this.feederWheel = new TalonFX(feederWheelId);
        this.robotStateSupplier = robotStateSupplier;
        this.intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        this.feederWheel.setNeutralMode(NeutralModeValue.Coast);
        intakeMotor.setPosition(0, 1.0);
        this.intakeMotor.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        var mm = configs.MotionMagic;
        mm.MotionMagicCruiseVelocity = 15; // 80 rotations per second
        mm.MotionMagicAcceleration = 7;   // Reach top speed in 0.5 seconds
        mm.MotionMagicJerk = 0;         // S-Curve smoothing
        var slot0 = configs.Slot0;
        slot0.kP = 1.4;     // An error of 2.5 rotations results in 12V output
        slot0.kV = 0.0;    // 1 rps requires 0.12V
        slot0.kS = 0.0;    // 0.25V to overcome static friction

        intakeMotor.getConfigurator().apply(configs);


    }
    @Override
    public void periodic(){
        // if(robotStateSupplier.get()==RobotState.INTAKE) {
        //     intakeMotor.setPosition(Constants.INTAKE_DOWN_POSITION);
        //     feederWheel.setVoltage(10);
        // } else if(robotStateSupplier.get()==RobotState.OUTTAKE) {
        //     feederWheel.setVoltage(-10);
        // } else{ //neutral, account for manual control
        //     intakeMotor.setVoltage(0);

        //     // if manual feeder, 
        // }
    }
}
