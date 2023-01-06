package frc.auton.mode;

import frc.auton.AutonOverride;
import frc.auton.RobotComponent;
import frc.auton.drive.DriveOutput;
import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveWait;
import frc.auton.intake.IntakeSetState;
import frc.auton.intake.IntakeWait;
import frc.auton.shooter.ShooterSetState;
import frc.auton.util.AutonWait;
import frc.subsystems.Intake.IntakeState;
import frc.subsystems.Shooter.ShooterState;

public class FenderLeft implements AutonMode{
    @Override
	public void addToMode(AutonBuilder ab) {
		// (really, do nothing, it's ok)
		ab.addCommand(new DriveSetPosition(0, 0, 21));
    
		ab.addCommand(new DriveWait());
        ab.addCommand(new ShooterSetState(ShooterState.FENDER));
		ab.addCommand(new AutonWait(2000));

		ab.addCommand(new IntakeSetState(IntakeState.SHOOTING_UP));
		ab.addCommand(new AutonWait(2000));

		ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
		ab.addCommand(new IntakeSetState(IntakeState.OFF_UP));
		ab.addCommand(new AutonOverride(RobotComponent.SHOOTER));
		ab.addCommand(new ShooterSetState(ShooterState.LAUNCH_PAD));
		ab.addCommand(new AutonWait(2000));
		ab.addCommand(new AutonOverride(RobotComponent.SHOOTER));
		ab.addCommand(new ShooterSetState(ShooterState.OFF));
		ab.addCommand(new AutonWait(5000));
		ab.addCommand(new DriveOutput(-0.4, 100000));
		ab.addCommand(new AutonWait(2000));
		ab.addCommand(new AutonOverride(RobotComponent.DRIVE));
		ab.addCommand(new DriveOutput(0.0, 100000));
		ab.addCommand(new AutonWait(1000));
		ab.addCommand(new IntakeWait());
		
		
	}
}
