/*
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
*/

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public Calculations calc;
    private TalonFX shooterMotor;
    private TalonFX hoodMotor;
    private final static double FLYWHEEL_RADIUS=0.06;

    public Shooter(Drivetrain drivetrain, int shooterMotorID, int hoodMotorID) {
        calc=new Calculations(drivetrain);

        shooterMotor = new TalonFX(shooterMotorID);
        shooterMotor.setPosition(0);

        hoodMotor = new TalonFX(hoodMotorID);
        hoodMotor.setPosition(0);
    }

    public TalonFX getShooterMotor() {
        return shooterMotor;
    }
    public TalonFX getHoodMotor() {
        return hoodMotor;
    }

    //Calculation methods
    /**
     * Used for defining the rotation of the chassis to compensate
     * shooting while moving, orientation ERROR INCLUDED
     */
    public double getCompensatoryRotation(){
        double sum=calc.orientationCorrectionWhileMoving()/*+calc.orientationCorrectionWhileMoving()*/;
        if(sum<-180){
            return sum+360;
        }else if(sum>180){
            return sum-360;
        }
        return sum;
    }

    public double getBallLaunchAngle(){
        return calc.angle();
    }
    public double getBallLaunchSpeed(){
        return calc.initialSpeed();
    }
    public double getTangentialVelocity(){
        return calc.tangentialVelocity();
    }
    public double getRadialVelocity(){
        return calc.radialVelocity();
    }
    
    //Conversion methods
    public static double ballspeedtoflywheel(double ballspeed){
        return ballspeed/FLYWHEEL_RADIUS/2/Math.PI;
    }
    public static double angletohood(double angle){        
        return (90-angle)/360;
    }

    public double getOrientationError() {
        return calc.orientationCorrectionWhileMoving();
    }
}

class Calculations{
    private final Drivetrain drivetrain;
    private final double timeToShoot;

    public Calculations(Drivetrain drivetrain){
        this.drivetrain=drivetrain;
        timeToShoot = 1.0;
    }

    //all in meters
    final static double RELATIVE_HEIGHT=0.48; //Assuming shooting height is constant, the hub height minus shooter height
    final static double FIELD_LENGTH=16.54;
    final static double FIELD_WIDTH=8.07;
    final static double BLUE_HUB_X=4.625;
    final static double RED_HUB_X=11.915;
    final static double HUB_Y=4.035;

    /**
     * Returns the distance from the hub
     * @return distance
     */
    // public double distance(){
    //     double x=drivetrain.swerveDrive.getPose().getX();
    //     double y=drivetrain.swerveDrive.getPose().getY();
    //     if(DriverStation.getAlliance().get()==DriverStation.Alliance.Blue)
    //         return Math.sqrt((x-BLUE_HUB_X)*(x-BLUE_HUB_X)+(y-HUB_Y)*(y-HUB_Y));
    //     else if(DriverStation.getAlliance().get()==DriverStation.Alliance.Red)
    //         return Math.sqrt((x-RED_HUB_X)*(x-RED_HUB_X)+(y-HUB_Y)*(y-HUB_Y));
    //     System.out.println("Invalid alliance");
    //     return 0;
    // }

    public double velocityX(){
        return drivetrain.swerveDrive.getFieldVelocity().vxMetersPerSecond;
    }

    public double velocityY(){
        return drivetrain.swerveDrive.getFieldVelocity().vyMetersPerSecond;
    }

    /**
     * From regression, returns the angle of the projectile IN DEGREES
     * that gets the ball into the hub in the shortest amount of time based on distance
     * from hub
     * @return if distance is less than 0.82 meters away, returns 80 DEGREES.
     * otherwise, returns an angle calculated by a regression IN DEGREES
     */
    public double angle(){
        double distance=drivetrain.distance();
        if(distance<0.82){
            return 80;
        }
        return 52.3332855296*Math.pow(0.612787696858,distance)+45;
    }

    /**
     * Returns the initial speed of the projectile that is time-optimal
     * given the distance from hub (accounts for shooting while moving)
     * @return the initial speed calculated from kinematics and based on the method angle
     */
    public double initialSpeed(){
        double desiredShooterSpeed2D=desiredShooterSpeed2D();
        if(desiredShooterSpeed2D<0){
            return 0;
        }
        double shooterVelocityX=-velocityX(); //without z
        double shooterVelocityY=desiredShooterSpeed2D-velocityY(); //without z
        double shooterSpeed2D=Math.sqrt(Math.pow(shooterVelocityX,2)+Math.pow(shooterVelocityY,2));
        return Math.sqrt(Math.pow(desiredShooterSpeed2D/(Math.cos(Math.toRadians(angle())))*Math.sin(Math.toRadians(angle())),2)+Math.pow(shooterSpeed2D,2));
    }

    public double desiredShooterSpeed2D(){
        double distance=drivetrain.distance() + radialVelocity() * timeToShoot;
        double theta=Math.toRadians(angle());
        //return (4.903325*distance*distance)/((Math.tan(theta)*distance-relativeHeight)*Math.cos(theta)*Math.cos(theta));
        // return Math.sqrt((-4.903325*distance*distance)/(relativeHeight*Math.cos(theta)*Math.cos(theta))+Math.tan(theta)*distance);
        double g = 9.80665;
        double numerator = g * distance * distance;
        double denominator = 2 * Math.pow(Math.cos(theta), 2) * (distance * Math.tan(theta) - RELATIVE_HEIGHT);
        if(denominator<=0){
            System.out.println("desiredShooterSpeed2D denominator=0");
            return -1;
        }
        return Math.sqrt(numerator / denominator)*Math.cos(Math.toRadians(angle()));
    }

    /**
     * Finds the tangential velocity of the bot. Note that autolock ensures that the bot will always be facing the hub
     * @return the velocity of the bot in the direction of the bot's right
     */
    public double tangentialVelocity(){
        double x=drivetrain.swerveDrive.getPose().getX();
        double y=drivetrain.swerveDrive.getPose().getY();
        double relX;
        double relY;
        double vx=velocityX();
        double vy=velocityY();
        
        if(DriverStation.getAlliance().get()==DriverStation.Alliance.Blue){
            relX=x-BLUE_HUB_X;
        }else if(DriverStation.getAlliance().get()==DriverStation.Alliance.Red){
            relX=x-RED_HUB_X;
        }else{
            return 0;
        }
        relY=y-HUB_Y;

        return Math.sqrt(vx*vx+vy*vy)*Math.sin(Math.atan2(vy,vx)-Math.atan2(-relY,-relX));
    }

    /**
     * Finds the radial velocity of the bot. Note that autolock ensures that the bot will always be facing the hub
     * @return the velocity of the bot in the direction of the bot's front
     */
    public double radialVelocity(){
        double x=drivetrain.swerveDrive.getPose().getX();
        double y=drivetrain.swerveDrive.getPose().getY();
        double relX;
        double relY;
        double vx=velocityX();
        double vy=velocityY();

        if(DriverStation.getAlliance().get()==DriverStation.Alliance.Blue){
            relX=x-BLUE_HUB_X;
        }else if(DriverStation.getAlliance().get()==DriverStation.Alliance.Red){
            relX=x-RED_HUB_X;
        }else{
            return 0;
        }
        relY=y-HUB_Y;

        return Math.sqrt(vx*vx+vy*vy)*Math.cos(Math.atan2(vy,vx)-Math.atan2(-relY,-relX));
    }

    /**
     * Finds the correction angle of the shooter IN DEGREES to account for the robot shooting while moving
     * @return the angle IN DEGREES that the shooter has to correct to account for robot's velocity
     */
    public double orientationCorrectionWhileMoving(){
        /*double desired2D=desiredShooterSpeed2D();
        double x=drivetrain.swerveDrive.getPose().getX();
        double y=drivetrain.swerveDrive.getPose().getY();
        double relX;
        double relY;
        if(DriverStation.getAlliance().get()==DriverStation.Alliance.Blue){
            relX=x-BLUE_HUB_X;
        }else if(DriverStation.getAlliance().get()==DriverStation.Alliance.Red){
            relX=x-RED_HUB_X;
        }else{
            return 0;
        }
        relY=y-HUB_Y;

        double desiredOrientation=Math.atan2(-relY,-relX);
        double shooterVelocityX=-velocityX(); //without z
        double shooterVelocityY=desired2D-velocityY(); //without z
        double actualOrientation=Math.atan2(shooterVelocityY,shooterVelocityX);
        double diff=Math.toDegrees(desiredOrientation-actualOrientation);
        if(diff<-180){
            return diff+360;
        }else if(diff>180){
            return diff-360;
        }
        return diff;*/
        /*
        double vx = tangentialVelocity();
        double vy = radialVelocity();

        double idealVelocity = desiredShooterSpeed2D();
        double angle = drivetrain.hubAngle();
        double idealVelocityX = idealVelocity * Math.cos(angle);
        double idealVelocityY = idealVelocity * Math.sin(angle);

        double realVelocityX = idealVelocityX - vx;
        double realVelocityY = idealVelocityY - vy;
        return Math.atan2(realVelocityY, realVelocityX);
        */

        double idealVelocity = desiredShooterSpeed2D();
        double hubAngle = drivetrain.hubAngle();
        double idealVelocityX = idealVelocity * Math.cos(hubAngle);
        double idealVelocityY = idealVelocity * Math.sin(hubAngle);

        double time = (drivetrain.distance() + tangentialVelocity()*timeToShoot) / (idealVelocity*Math.cos(angle()));
        double distanceBotMoves = tangentialVelocity() * time;
        double angleError = Math.atan2(distanceBotMoves, drivetrain.distance());
        return 0.0;
    }

    /**
     * Finds the error in orientation of the bot based on position and actual orientation
     * @return real orientation minus expected orientation IN DEGREES
     */
    // public double orientationError(){
    //     double x=drivetrain.swerveDrive.getPose().getX();
    //     double y=drivetrain.swerveDrive.getPose().getY();
    //     double relX;
    //     double relY;
    //     if(DriverStation.getAlliance().get()==DriverStation.Alliance.Blue){
    //         relX=x-BLUE_HUB_X;
    //     }else if(DriverStation.getAlliance().get()==DriverStation.Alliance.Red){
    //         relX=x-RED_HUB_X;
    //     }else{
    //         return 0;
    //     }
    //     relY=y-HUB_Y;

    //     double error=drivetrain.swerveDrive.getPose().getRotation().getDegrees()-Math.toDegrees(Math.atan2(-relY,-relX));
    //     if(error<-180){
    //         return error+360;
    //     }else if(error>180){
    //         return error-360;
    //     }
    //     return error;
    // }
}