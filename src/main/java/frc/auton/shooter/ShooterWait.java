package frc.auton.shooter;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;

public class ShooterWait extends AutonCommand {

	public ShooterWait() {
		super(RobotComponent.SHOOTER);
	}

	@Override
	public boolean calculate() {
		return true;
	}

	@Override
	public void override() {
		// TODO Auto-generated method stub

	}

	@Override
	public void firstCycle() {
		// TODO Auto-generated method stub

	}

}