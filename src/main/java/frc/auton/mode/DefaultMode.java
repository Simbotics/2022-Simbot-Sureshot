package frc.auton.mode;

import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveWait;
import frc.auton.util.RunFirstCycle;

/**
 *
 * @author Michael
 */
public class DefaultMode implements AutonMode {

	@Override
	public void addToMode(AutonBuilder ab) {
		// (really, do nothing, it's ok)
		ab.addCommand(new DriveSetPosition(0, 0, 0));
		ab.addCommand(new DriveWait());
		
		
	}
}
