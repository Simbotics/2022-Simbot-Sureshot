package frc.auton.mode;

import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveTurnToAngle;
import frc.auton.drive.DriveWait;
import frc.auton.util.RunFirstCycle;

/**
 *
 * @author Michael
 */
public class TurnTo90 implements AutonMode {

	@Override
	public void addToMode(AutonBuilder ab) {
		// (really, do nothing, it's ok)
		ab.addCommand(new DriveSetPosition(0, 0, 0));
		ab.addCommand(new DriveTurnToAngle(20, 1, 0.5, 5000));
		ab.addCommand(new DriveWait());
		
		
	}
}
