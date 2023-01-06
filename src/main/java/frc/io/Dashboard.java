package frc.io;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auton.mode.AutonMode;
import frc.imaging.SimLimelight;
import frc.robot.RobotConstants;
import frc.util.PIDConstants;
import frc.util.ProfileConstants;
import frc.util.TrajectoryConfig;

//The Dashboard prints everything and every value that is important on to the dashboard
public class Dashboard {
	private static Dashboard instance;
	private boolean manual = false;



	private Dashboard() {
		
		
		
	}

	public static Dashboard getInstance() {
		if (instance == null) {
			instance = new Dashboard();
		}
		return instance;
	}

	public void updateAll() {
		updateSensorDisplay();
	}

	public void updateSensorDisplay() {

		
		//SmartDashboard.putNumber("MATCH TIME: ", this.io.getMatchTimeLeft());
	}

	public double getConstant(String name, double defaultValue) {
		return SmartDashboard.getNumber(name, defaultValue);
	}

	// Get the PID Constants
	public PIDConstants getPIDConstants(String name, PIDConstants constants) {
		double p = SmartDashboard.getNumber("5_" + name + " - P Value", constants.p);
		double i = SmartDashboard.getNumber("5_" + name + " - I Value", constants.i);
		double d = SmartDashboard.getNumber("5_" + name + " - D Value", constants.d);
		double ff = SmartDashboard.getNumber("5_" + name + " - FF Value", constants.ff);
		double eps = SmartDashboard.getNumber("5_" + name + " - EPS Value", constants.eps);
		return new PIDConstants(p, i, d, ff, eps);
	}

	// Put the PID Constants on the dashboard
	public void putPIDConstants(String name, PIDConstants constants) {
		SmartDashboard.putNumber("5_" + name + " - P Value", constants.p);
		SmartDashboard.putNumber("5_" + name + " - I Value", constants.i);
		SmartDashboard.putNumber("5_" + name + " - D Value", constants.d);
		SmartDashboard.putNumber("5_" + name + " - FF Value", constants.ff);
		SmartDashboard.putNumber("5_" + name + " - EPS Value", constants.eps);
	}

	public ProfileConstants getProfileConstants(String name, ProfileConstants constants) {
		double p = SmartDashboard.getNumber("5_" + name + " - P Value", constants.p);
		double i = SmartDashboard.getNumber("5_" + name + " - I Value", constants.i);
		double d = SmartDashboard.getNumber("5_" + name + " - D Value", constants.d);
		double vFF = SmartDashboard.getNumber("3_" + name + " - vFF Value", constants.vFF);
		double aFF = SmartDashboard.getNumber("3_" + name + " - aFF Value", constants.aFF);
		double dFF = SmartDashboard.getNumber("3_" + name + " - dFF Value", constants.dFF);
		double gFF = SmartDashboard.getNumber("3_" + name + " - gFF Value", constants.gravityFF);
		double posEps = SmartDashboard.getNumber("3_" + name + " - Pos EPS Value", constants.positionEps);
		double velEps = SmartDashboard.getNumber("3_" + name + " - Vel EPS Value", constants.velocityEps);
		return new ProfileConstants(p, i, d, vFF, aFF, dFF, gFF, posEps, velEps);
	}

	// Put all the profile constants on the smart dashboard
	public void putProfileConstants(String name, ProfileConstants constants) {
		SmartDashboard.putNumber("5_" + name + " - P Value", constants.p);
		SmartDashboard.putNumber("5_" + name + " - I Value", constants.i);
		SmartDashboard.putNumber("5_" + name + " - D Value", constants.d);
		SmartDashboard.putNumber("3_" + name + " - vFF Value", constants.vFF);
		SmartDashboard.putNumber("3_" + name + " - aFF Value", constants.aFF);
		SmartDashboard.putNumber("3_" + name + " - dFF Value", constants.dFF);
		SmartDashboard.putNumber("3_" + name + " - gFF Value", constants.gravityFF);
		SmartDashboard.putNumber("3_" + name + " - Pos EPS Value", constants.positionEps);
		SmartDashboard.putNumber("3_" + name + " - Vel EPS Value", constants.velocityEps);
	}

	// Get the acceleration and velocity values
	public TrajectoryConfig getTrajectoryConfig(String name, TrajectoryConfig constants) {
		double maxAccel = SmartDashboard.getNumber("3_" + name + " - Max Accel Value", constants.maxAcceleration);
		double maxDecel = SmartDashboard.getNumber("3_" + name + " - Max Decel Value", constants.maxDeceleration);
		double maxVel = SmartDashboard.getNumber("3_" + name + " - Max Vel Value", constants.maxVelocity);
		return new TrajectoryConfig(maxAccel, maxDecel, maxVel);

	}

	

	// Put all the acceleration and velocity values on screen
	public void putTrajectoryConfig(String name, TrajectoryConfig constants) {
		SmartDashboard.putNumber("3_" + name + " - Max Accel Value", constants.maxAcceleration);
		SmartDashboard.putNumber("3_" + name + " - Max Decel Value", constants.maxDeceleration);
		SmartDashboard.putNumber("3_" + name + " - Max Vel Value", constants.maxVelocity);

	}

}
