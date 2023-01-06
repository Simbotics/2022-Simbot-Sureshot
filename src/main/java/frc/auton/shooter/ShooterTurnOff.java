package frc.auton.shooter;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.robot.RobotConstants;

public class ShooterTurnOff extends AutonCommand {

    private IO io;
    

    public ShooterTurnOff() {
        super(RobotComponent.SHOOTER);
        this.io = IO.getInstance();
        
    }

    @Override
    public void firstCycle() {
    }

    @Override
    // Sets the motor outputs
    public boolean calculate() {
       this.io.setShooterBottomMotor(0);
       this.io.setShooterTopMotor(0);

       return true;
        
    }

    @Override
    // When activated, stops the robot
    public void override() {
        this.io.setShooterBottomMotor(0);
        this.io.setShooterTopMotor(0);
        
    }

}