package frc.auton.mode;

import frc.auton.AutonOverride;
import frc.auton.RobotComponent;
import frc.auton.drive.DriveOutput;
import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveTurnToLimelight;
import frc.auton.drive.DriveWait;
import frc.auton.intake.IntakeSetState;
import frc.auton.intake.IntakeWait;
import frc.auton.shooter.ShooterSetState;
import frc.auton.util.AutonWait;
import frc.subsystems.Intake.IntakeState;
import frc.subsystems.Shooter.ShooterState;

public class DriveBackShootPreload implements AutonMode{
    @Override
	public void addToMode(AutonBuilder ab) {
		// (really, do nothing, it's ok)
		ab.addCommand(new DriveSetPosition(0, 0, 0));
    
		ab.addCommand(new DriveWait());
        ab.addCommand(new ShooterSetState(ShooterState.LIMELIGHT_SHOOTING));
		ab.addCommand(new DriveOutput(-0.4, 100000));
		ab.addCommand(new AutonWait(1000));
		ab.addCommand(new AutonOverride(RobotComponent.DRIVE));
		ab.addCommand(new DriveOutput(0.0, 100000));
		ab.addCommand(new AutonWait(1000));
		ab.addCommand(new AutonOverride(RobotComponent.DRIVE));
		ab.addCommand(new DriveTurnToLimelight(0.0, 3, 3000));
		
		ab.addCommand(new DriveWait());
		ab.addCommand(new IntakeSetState(IntakeState.SHOOTING_UP));
		ab.addCommand(new AutonWait(2000));

		ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
		ab.addCommand(new IntakeSetState(IntakeState.OFF_UP));
		ab.addCommand(new AutonOverride(RobotComponent.SHOOTER));
		ab.addCommand(new ShooterSetState(ShooterState.OFF));
		ab.addCommand(new IntakeWait());
		
		
	}
}

