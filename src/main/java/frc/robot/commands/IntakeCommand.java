package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer.RobotState;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command{
    Intake intake;
    Supplier<Boolean> isManual;
    Supplier<RobotState> getState;
    Supplier<RobotState> getFeederWheelSpeedsManual;
    //-1 to go down, 0 to stay still, 1+ to move up
    Supplier<Double> getRotationalSpeedDirection;
    PIDController intakeController;
    double manualSetpoint;
    boolean lastAutomatic = true;
    public IntakeCommand(
        Intake intake,
        Supplier<Boolean> isManual,
        Supplier<RobotState> getState,
        Supplier<RobotState> getFeederWheelSpeedsManual,   
        Supplier<Double> getRotationalSpeedDirection,    
        double kP,
        double kI,
        double kD
    ){
        this.intake = intake;
        this.getState = getState;
        this.isManual = isManual;
        this.getFeederWheelSpeedsManual = getFeederWheelSpeedsManual;
        this.getRotationalSpeedDirection = getRotationalSpeedDirection ;
        this.intakeController = new PIDController(kP, kI, kD);

    }
    public Set<Subsystem> getRequirements(){
        Set<Subsystem> x = new HashSet<Subsystem>();
        x.add(intake);
        return x;        
    }
    public void execute(){
        SmartDashboard.putBoolean("Manual Intake", isManual.get());
        // System.out.println(intake.intakeMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("sdfdsfsdfdsfsd", intake.intakeMotor.getPosition().getValueAsDouble());

        if(isManual.get() ){
            // deal with actual wrist
            // if(intake.intakeMotor.getPosition().getValueAsDouble()>-.04 || intake.intakeMotor.getPosition().getValueAsDouble()<Constants.INTAKE_DOWN_POSITION+.04 )
            intake.intakeMotor.set((getRotationalSpeedDirection.get()>=0?getRotationalSpeedDirection.get()*.1:-.1));
            
            //deal with feeder
            if(getFeederWheelSpeedsManual.get()==RobotState.INTAKE ||getFeederWheelSpeedsManual.get()==RobotState.OUTTAKE ) 
                intake.feederWheel.set(Constants.MAX_FLYWHEEL_VOLTAGE*(getFeederWheelSpeedsManual.get()==RobotState.INTAKE?1:-1));
            else
                intake.feederWheel.set(0);
        } else {
            SmartDashboard.putNumber("current motor position", intake.intakeMotor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("setpoint",correspondingSetpoint());

            // intake.intakeMotor.set(-intakeController.calculate(intake.intakeMotor.getPosition().getValueAsDouble(), correspondingSetpoint()));
            if(getState.get()!=RobotState.NEUTRAL){
                intake.intakeMotor.setControl(new MotionMagicDutyCycle(correspondingSetpoint()));
                // System.out.println();
                SmartDashboard.putNumber("CORRESPONDING ANGLE", correspondingSetpoint());
            }
            if(getState.get()!=RobotState.NEUTRAL && getState.get()!=RobotState.DEFENSE && (Math.abs(-5.5-intake.intakeMotor.getPosition().getValueAsDouble()))<.7) 
                intake.feederWheel.set(Constants.MAX_FLYWHEEL_VOLTAGE*(getState.get()==RobotState.INTAKE?1:-1));
            else 
                intake.feederWheel.set(0);
        }
    }
    private double correspondingSetpoint(){
        SmartDashboard.putString("state", getState.get().toString());
        if(getState.get()==RobotState.INTAKE || getState.get()==RobotState.OUTTAKE/* || desiredState!=RobotState.HYBRID*/){
            return Constants.INTAKE_DOWN_POSITION;
        } else if (getState.get()==RobotState.DEFENSE){
            return 0;
        } else {
            return intake.intakeMotor.getPosition().getValueAsDouble();
        }
    }
}