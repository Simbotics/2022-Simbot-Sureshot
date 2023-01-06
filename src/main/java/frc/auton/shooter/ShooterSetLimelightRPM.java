package frc.auton.shooter;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.imaging.SimLimelight.LimelightTargetType;
import frc.io.IO;
import frc.robot.RobotConstants;
import frc.subsystems.Shooter;
import frc.subsystems.Shooter.ShooterState;

public class ShooterSetLimelightRPM extends AutonCommand {

    private IO io;
    private double topTarget;
    private double bottomTarget;
    
    private Shooter shooter;

    public ShooterSetLimelightRPM(double topRPM, double bottomRPM, long timeout) {
        super(RobotComponent.SHOOTER,timeout);
        this.io = IO.getInstance();
        this.shooter = Shooter.getInstance();
        if(topRPM > RobotConstants.TOP_SHOOTER_MAX_RPM){
            topRPM = RobotConstants.TOP_SHOOTER_MAX_RPM;
        }

        if(bottomRPM > RobotConstants.BOTTOM_SHOOTER_MAX_RPM){
            bottomRPM = RobotConstants.BOTTOM_SHOOTER_MAX_RPM;
        }
        this.topTarget = topRPM;
        this.bottomTarget = bottomRPM;
        
    }

    @Override
    public void firstCycle() {
        this.shooter.setState(ShooterState.LIMELIGHT_SHOOTING);
    }

    @Override
    // Sets the motor outputs
    public boolean calculate() {
        this.io.setLimelightTarget(LimelightTargetType.GOAL);

        if(this.io.getVisionTargetExists()){
            this.shooter.calculate();
            if(this.shooter.isShooterAtSpeed()){
                double distance = this.io.getVisionDistanceFeet();
                System.out.println("LIMELIGHT DISTANCE: " + distance);
                System.out.println("LIMELIGHT TOP RPM: " + this.shooter.shooterTopRPMBasedOnDistance(distance));
                System.out.println("LIMELIGHT BOTTOM RPM: " + this.shooter.shooterBottomRPMBasedOnDistance(distance));
                return true;
            } else{
                return false;
            }
            

        } else{
            this.io.setShooterTopRPM(this.topTarget);
            this.io.setShooterBottomRPM(this.bottomTarget);
            return false;
        }

       
       
        
    }

    @Override
    // When activated, stops the robot
    public void override() {
        
    }

}