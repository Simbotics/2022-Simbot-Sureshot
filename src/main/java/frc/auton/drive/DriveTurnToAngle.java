package frc.auton.drive;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.robot.RobotConstants;
import frc.subsystems.Drive;
import frc.util.SimLib;
import frc.util.SimPID;

public class DriveTurnToAngle extends AutonCommand {

	private IO io;
	private double targetAngle;
	private double eps; // Acceptable range
	private SimPID turnPID;
	private double maxOutput;
	private Drive drive;

	// Declares needed variables
	public DriveTurnToAngle(double targetAngle, double eps, long timeoutLength) {
		this(targetAngle, 1, eps, timeoutLength);
	}



	// Declares needed variables, the maxOutput and the rampRate
	public DriveTurnToAngle(double targetAngle, double maxOutput,  double eps, long timeoutLength) {
		super(RobotComponent.DRIVE, timeoutLength);
		this.targetAngle = targetAngle;
		this.drive = Drive.getInstance();
		this.maxOutput = maxOutput;
		this.eps = eps;
		this.io = IO.getInstance();

	}

	@Override
	public void firstCycle() {
		
		this.turnPID = new SimPID(RobotConstants.getHeadingPID());
		this.turnPID.setMaxOutput(this.maxOutput);
		this.turnPID.setFinishedRange(this.eps);
		this.turnPID.setDesiredValue(this.targetAngle);
		this.turnPID.setIRange(2);
		this.io.setDriveBrakeMode(true);
		
	}

	@Override
	// Sets the motor outputs for turning
	public boolean calculate() {
		double turnOutput = this.turnPID.calcPID(this.io.getHeading());
		

		if (this.turnPID.isDone()) {
			this.io.drive(0, 0, 0);
			return true;
		} else {
			
            this.io.drive(0, 0, turnOutput);
			
			return false;
		}
	}

	@Override
	// When activated, stops the robot
	public void override() {
		this.io.drive(0, 0, 0);
	}

}