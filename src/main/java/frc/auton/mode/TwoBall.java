package frc.auton.mode;

import frc.auton.AutonOverride;
import frc.auton.RobotComponent;
import frc.auton.drive.DriveOutput;
import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveToPoint;
import frc.auton.drive.DriveToPointVisionAngle;
import frc.auton.drive.DriveTurnToLimelight;
import frc.auton.drive.DriveWait;
import frc.auton.intake.IntakeSetState;
import frc.auton.intake.IntakeWait;
import frc.auton.shooter.ShooterSetLimelightRPM;
import frc.auton.shooter.ShooterSetRPM;
import frc.auton.shooter.ShooterSetState;
import frc.auton.shooter.ShooterWait;
import frc.auton.util.AutonWait;
import frc.subsystems.Intake.IntakeState;
import frc.subsystems.Shooter.ShooterState;

public class TwoBall implements AutonMode{
    @Override
	public void addToMode(AutonBuilder ab) {
		// (really, do nothing, it's ok)
        ab.addCommand(new DriveSetPosition(0, 0, 43.5));
        ab.addCommand(new IntakeSetState(IntakeState.INTAKING_DOWN));
        ab.addCommand(new ShooterSetRPM(2100, 2100, 100, 2000));
		ab.addCommand(new DriveToPointVisionAngle(-0.80, -0.80, 30.0, 0.0, 0.09, 5000));
        ab.addCommand(new DriveTurnToLimelight(30, 1.0, 3000));
        ab.addCommand(new ShooterSetLimelightRPM(2100, 2100, 2000));
        ab.addCommand(new DriveWait());
        ab.addCommand(new ShooterWait());

        ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
        ab.addCommand(new IntakeSetState(IntakeState.SHOOTING_DOWN));

        ab.addCommand(new AutonWait(1000));
        ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
        ab.addCommand(new IntakeSetState(IntakeState.OFF_DOWN));

        
        ab.addCommand(new AutonWait(1000));
        ab.addCommand(new DriveToPointVisionAngle(-0.95, -0.95, 30.0, 0.0, 0.09, 5000));
        ab.addCommand(new DriveWait());
       
      
      

        





        
		
		
		
	}
}

