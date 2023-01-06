package frc.auton.drive;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.Dashboard;
import frc.io.IO;
import frc.subsystems.Drive;


public class DriveToPoint extends AutonCommand {

	private double x;
	private double y;
	private double theta; // The desired angle
	private double minVelocity;
	private double maxVelocity;
	private double eps; // The acceptable range
    private double turnSpeed;
	private IO io;

	private Drive drive;

	public DriveToPoint(double x, double y, double theta, long timeout) {
		this(x, y, theta, 0.0, 0.03, timeout);
	}

	public DriveToPoint(double x, double y, double theta, double minVelocity, double eps, long timeout) {
		this(x, y, theta, minVelocity, 1.0, eps, timeout);
	}

    public DriveToPoint(double x, double y, double theta, double minVelocity, double maxVelocity, double eps,  long timeout){
        this(x, y, theta, minVelocity, maxVelocity, 1.0, eps, timeout);
    }
	
	

	public DriveToPoint(double x, double y, double theta, double minVelocity, double maxVelocity, double turnSpeed,
			double eps,  long timeout) {
		super(RobotComponent.DRIVE, timeout);
		this.x = x;
		this.y = y;
		this.theta = theta;
		this.minVelocity = minVelocity;
		this.maxVelocity = maxVelocity;
		this.eps = eps;
        this.turnSpeed = turnSpeed;

		
        this.io = IO.getInstance();
		this.drive = Drive.getInstance();
		this.io.setDriveBrakeMode(true);
		
		
	}

	@Override
	public void firstCycle() {
		
		

	}

	@Override
	public boolean calculate() {
		boolean isDone =  this.drive.DriveToPoint(x, y, theta, minVelocity, maxVelocity, turnSpeed, eps);
		return isDone;
		
	}

	@Override
	public void override() {
        
		this.io.drive(0, 0, 0);

	}

}
