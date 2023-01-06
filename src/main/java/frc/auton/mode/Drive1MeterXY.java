package frc.auton.mode;

import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveToPoint;
import frc.auton.drive.DriveTurnToAngle;
import frc.auton.drive.DriveWait;
import frc.auton.util.RunFirstCycle;

/**
 *
 * @author Michael
 */
public class Drive1MeterXY implements AutonMode {

	@Override
	public void addToMode(AutonBuilder ab) {
		// (really, do nothing, it's ok)
		ab.addCommand(new DriveSetPosition(0, 0, 0));
		ab.addCommand(new DriveToPoint(1, 1, 25, 0, 1, 1, 0.01, 5000));
		ab.addCommand(new DriveToPoint(0, 0, 0, 0, 1, 1, 0.01, 5000));
		ab.addCommand(new DriveWait());
		
		
	}
}
