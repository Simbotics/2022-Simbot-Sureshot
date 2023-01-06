package frc.auton.mode;

import frc.auton.AutonOverride;
import frc.auton.RobotComponent;
import frc.auton.drive.DriveOutput;
import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveToPoint;
import frc.auton.drive.DriveToPointVisionAngle;
import frc.auton.drive.DriveTurnToAngle;
import frc.auton.drive.DriveTurnToLimelight;
import frc.auton.drive.DriveWait;
import frc.auton.intake.IntakeSetState;
import frc.auton.intake.IntakeWait;
import frc.auton.intake.IntakeWaitForBall;
import frc.auton.shooter.ShooterSetLimelightRPM;
import frc.auton.shooter.ShooterSetRPM;
import frc.auton.shooter.ShooterSetState;
import frc.auton.shooter.ShooterWait;
import frc.auton.util.AutonWait;
import frc.subsystems.Intake.IntakeState;
import frc.subsystems.Shooter.ShooterState;

public class FiveBallStemley implements AutonMode{
    @Override
	public void addToMode(AutonBuilder ab) {
		// (really, do nothing, it's ok)
        ab.addCommand(new DriveSetPosition(0, 0, -89));
        ab.addCommand(new IntakeSetState(IntakeState.INTAKING_DOWN));
        ab.addCommand(new ShooterSetRPM(2000, 2000, 100, 2000));
		ab.addCommand(new DriveToPointVisionAngle(0.0, 1.08, -73, 0.0, 0.09, 5000));
        ab.addCommand(new DriveTurnToLimelight(-73, 1.0, 1500));
        ab.addCommand(new ShooterSetLimelightRPM(2000, 2000, 2000));
        ab.addCommand(new DriveWait());
        ab.addCommand(new ShooterWait());

        ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
        ab.addCommand(new IntakeSetState(IntakeState.SHOOTING_DOWN));

        ab.addCommand(new AutonWait(700));
        ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
        ab.addCommand(new IntakeSetState(IntakeState.INTAKING_DOWN));
       
      
       

        ab.addCommand(new DriveToPoint(-2.75, -0.1, 30, 0.0, 0.8,0.1, 5000));
        ab.addCommand(new DriveTurnToLimelight(-35, 1.0, 1500));
        ab.addCommand(new ShooterSetLimelightRPM(2000, 2000, 2000));
       
        ab.addCommand(new DriveWait());
        ab.addCommand(new ShooterWait());

        ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
        ab.addCommand(new IntakeSetState(IntakeState.SHOOTING_DOWN));

        ab.addCommand(new AutonWait(850));
        ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
        ab.addCommand(new IntakeWaitForBall(5000));
        
        ab.addCommand(new DriveToPoint(-6.8, 0.1, -30, 0.0, 0.85,0.1, 5000));
        ab.addCommand(new DriveToPoint(-6.5, -0.05, -30, 0.0, 0.4,0.1, 5000));
        ab.addCommand(new DriveWait());
        ab.addCommand(new IntakeWait());

        ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
        ab.addCommand(new IntakeSetState(IntakeState.OFF_DOWN_AUTO));


        ab.addCommand(new DriveToPointVisionAngle(-3.0, -0.50, -35, 0.0, 0.8,0.07, 5000));
        ab.addCommand(new ShooterSetState(ShooterState.LIMELIGHT_SHOOTING));
        ab.addCommand(new DriveTurnToAngle(-35, 10, 500));
        ab.addCommand(new DriveTurnToLimelight(-35, 1.0, 1000));
        ab.addCommand(new AutonOverride(RobotComponent.SHOOTER));
        ab.addCommand(new ShooterSetLimelightRPM(3000, 3000, 2000));
        ab.addCommand(new DriveWait());
        ab.addCommand(new ShooterWait());

        ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
        ab.addCommand(new IntakeSetState(IntakeState.SHOOTING_DOWN));
        ab.addCommand(new AutonWait(1000));

        ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
        ab.addCommand(new IntakeSetState(IntakeState.OFF_DOWN));
        //ab.addCommand(new DriveToPoint(-3.5, -1 , 45 , 0.0, 0.1, 2000));
        ab.addCommand(new DriveToPoint(0.78, 0.8 , 180 , 0.0, 0.7, 0.01, 3000));
        ab.addCommand(new DriveWait());
        // x: -3.5 y: -1 O: 45
        // x: 0.68 y: 0.82 O: 223.4





        
		
		
		
	}
}
