package frc.auton.drive;

import edu.wpi.first.math.util.Units;
import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.robot.RobotConstants;
import frc.subsystems.Drive;
import frc.util.SimLib;
import frc.util.SimPID;

public class DriveTurnToLimelight extends AutonCommand {

	private IO io;
	private double targetAngle;
	private double eps; // Acceptable range
	private SimPID turnPID;
	private double maxOutput;
	private Drive drive;

	// Declares needed variables
	public DriveTurnToLimelight(double targetAngle, double eps, long timeoutLength) {
		this(targetAngle, 1, eps, timeoutLength);
	}



	// Declares needed variables, the maxOutput and the rampRate
	public DriveTurnToLimelight(double targetAngle, double maxOutput,  double eps, long timeoutLength) {
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
		this.turnPID.setIRange(5);
		this.io.setDriveBrakeMode(true);
		
	}

	@Override
	// Sets the motor outputs for turning
	public boolean calculate() {

		double turnOutput;

		if(this.io.getVisionTargetExists()){
			this.turnPID.setDesiredValue(this.io.getHeading() - this.io.getVisionTargetX() +  this.drive.getAimOffset());
			this.io.updateGoalPose(Units.feetToMeters( this.io.getVisionDistanceFeet()), this.io.getVisionTargetX());
			turnOutput = this.turnPID.calcPID(this.io.getHeading());
		} else { // go to preset value is we never see any vision target
			turnOutput = this.turnPID.calcPID(this.io.getHeading());
		}

		
		

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