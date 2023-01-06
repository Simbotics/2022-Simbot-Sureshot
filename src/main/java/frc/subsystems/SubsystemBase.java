package frc.subsystems;

import frc.io.DriverInput;
import frc.io.IO;

/*
 * Core of the entire subsystem. This class defines all the functions that need to exist in any
 * children, as well as defines the common input and output methods that get utilized in every
 * subsystem
 */
public abstract class SubsystemBase implements iSubsystem {

	
	protected IO io;
	protected DriverInput driverIn;

	/*
	 * Setup the base of our calculate function. Since we are in the abstract that every other
	 * Subsystem extends from, we want to do the common code here. Primarily, getting an active
	 * instance for all of our input and output systems
	 */
	public SubsystemBase() {
		this.io = IO.getInstance();
		this.driverIn = DriverInput.getInstance();

		this.firstCycle();
  	}
}
