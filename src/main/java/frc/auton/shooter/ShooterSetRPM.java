package frc.auton.shooter;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.robot.RobotConstants;

public class ShooterSetRPM extends AutonCommand {

    private IO io;
    private double topTarget;
    private double bottomTarget;
    private double eps;

    public ShooterSetRPM(double topRPM, double bottomRPM, double eps, long timeout) {
        super(RobotComponent.SHOOTER,timeout);
        this.io = IO.getInstance();
        if(topRPM > RobotConstants.TOP_SHOOTER_MAX_RPM){
            topRPM = RobotConstants.TOP_SHOOTER_MAX_RPM;
        }

        if(bottomRPM > RobotConstants.BOTTOM_SHOOTER_MAX_RPM){
            bottomRPM = RobotConstants.BOTTOM_SHOOTER_MAX_RPM;
        }
        this.topTarget = topRPM;
        this.bottomTarget = bottomRPM;
        this.eps = eps;
    }

    @Override
    public void firstCycle() {
    }

    @Override
    // Sets the motor outputs
    public boolean calculate() {
        this.io.setShooterTopRPM(this.topTarget);
        this.io.setShooterBottomRPM(this.bottomTarget);
        if((Math.abs(this.topTarget - this.io.getShooterTopRPM()) < this.eps) 
        && Math.abs(this.bottomTarget - this.io.getShooterBottomRPM()) < this.eps){
            return true;

        } else {
            return false;
        }
        
    }

    @Override
    // When activated, stops the robot
    public void override() {
        
    }

}