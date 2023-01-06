package frc.auton.drive;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.robot.RobotConstants;
import frc.subsystems.Drive;
import frc.util.SimLib;
import frc.util.SimPID;

public class DriveOutput extends AutonCommand {

	private IO io;
	
	private Drive drive;
    private double output; 



	// Declares needed variables, the maxOutput and the rampRate
	public DriveOutput(double output,  long timeoutLength) {
		super(RobotComponent.DRIVE, timeoutLength);
		
		this.drive = Drive.getInstance();
		this.output = output;
		this.io = IO.getInstance();

	}

	@Override
	public void firstCycle() {
		
		this.io.setFieldOriented(false);
		this.io.setDriveBrakeMode(true);
		
	}

	@Override
	// Sets the motor outputs for turning
	public boolean calculate() {
		this.io.drive(output, 0 , 0);
        return false;
	}

	@Override
	// When activated, stops the robot
	public void override() {
		this.io.drive(0, 0, 0);
	}

}