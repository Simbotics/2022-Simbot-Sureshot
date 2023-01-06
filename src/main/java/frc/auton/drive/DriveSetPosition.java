package frc.auton.drive;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.subsystems.Drive;
import frc.subsystems.Intake;

import frc.util.SimPoint;

public class DriveSetPosition extends AutonCommand {

	private double x;
	private double y;
	private double angle;
	private IO io;
	private Intake intake;
	private Drive drive;
	

	// Declares the variables for the position of the robot using SimPoints
	public DriveSetPosition(SimPoint p, double angle) {
		this(p.getX(), p.getY(), angle);
	}

	

	// Declares the variables for the regular way of setting the position of the
	// robot
	public DriveSetPosition(double x, double y, double angle) {
		super(RobotComponent.DRIVE);
		this.x = x;
		this.y = y;
		this.angle = angle;
		this.io = IO.getInstance();
		this.drive = Drive.getInstance();
		this.intake = Intake.getInstance();
	}

	@Override
	public void firstCycle() {
		this.io.setDrivePose(this.x, this.y,this.angle);
		//System.out.println("SETTING ANGLE" + this.angle);
		this.io.setDriveBrakeMode(true);
		this.intake.setColorSensorEnabled(false);
		this.drive.setColorSensorEnabled(false);
		this.drive.resetRateLimit();
		

	}

	@Override
	// Sets the position of the robot on the field
	public boolean calculate() {
		
		this.io.setDrivePose(this.x, this.y,this.angle);
		this.io.setDriveBrakeMode(true);
		
		
		

		return true;
	}

	@Override
	public void override() {

	}

}