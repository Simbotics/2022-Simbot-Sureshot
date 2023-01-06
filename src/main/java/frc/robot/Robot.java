package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.auton.AutonControl;
import frc.imaging.SimLimelight;
import frc.imaging.SimLimelight.LimelightTargetType;
import frc.io.Dashboard;
import frc.io.Logger;
import frc.io.IO;
import frc.teleop.TeleopControl;
import frc.teleop.TeleopController;
import frc.util.Debugger;

public class Robot extends TimedRobot {
	
	private IO io;
	private TeleopControl teleopControl;
	private Logger logger;
	private Dashboard dashboard;
	private boolean pushToDashboard = true;
	public static boolean teleopInitialized = false;
	private boolean haveWeZeroed = false;

	public Robot(){
		super(0.01);
	}

	@Override
	public void robotInit() {
		Debugger.defaultOn();
		this.io = IO.getInstance();
		this.dashboard = Dashboard.getInstance();
		if (this.pushToDashboard) {
			RobotConstants.pushValues();
		}
		
		
		this.teleopControl = TeleopControl.getInstance();
		
		this.logger = Logger.getInstance();
		LiveWindow.setEnabled(false);
		LiveWindow.disableAllTelemetry();
		this.teleopControl.initialize();
		this.io.setLimelightTarget(LimelightTargetType.DRIVING);
		

	}

	@Override
	public void robotPeriodic() {
		this.io.update();
		this.dashboard.updateAll();
	}

	@Override
	public void disabledInit() {
		this.io.stopAll();
		this.teleopControl.disable();
		this.logger.close();
		
	
	}

	
	@Override
	public void disabledPeriodic() {
		AutonControl.getInstance().updateModes();
		
	}
		
	


	@Override
	public void autonomousInit() {
		
		AutonControl.getInstance().initialize();
		AutonControl.getInstance().setRunning(true);
		AutonControl.getInstance().setOverrideAuto(false);
		this.io.reset();
		this.io.resetAutonTimer();
		this.logger.openFile();
		

	}

	@Override
	public void autonomousPeriodic() {
		AutonControl.getInstance().runCycle();
		this.logger.logAll();
	}

	public void testInit() {

	}

	public void testPeriodic() {

	}

	@Override
	public void teleopInit() {
		
		this.teleopControl.initialize();
		Robot.teleopInitialized = true;
	
	}

	@Override
	public void teleopPeriodic() {
		this.teleopControl.runCycle();
		
	}

	
}
