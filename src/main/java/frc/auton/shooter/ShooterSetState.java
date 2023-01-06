package frc.auton.shooter;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.subsystems.Intake;
import frc.subsystems.Shooter;
import frc.subsystems.Intake.IntakeState;

public class ShooterSetState extends AutonCommand {

    private IO io;
    private Shooter shooter;
    private Shooter.ShooterState shooterState;

    public ShooterSetState(Shooter.ShooterState shooterState){
        super(RobotComponent.SHOOTER);
        this.io = IO.getInstance();
        this.shooter = Shooter.getInstance();
        this.shooterState = shooterState;
    }

	@Override
	public void firstCycle() {
		shooter.setState(this.shooterState);
	}

	@Override
	// Sets the motor outputs
	public boolean calculate() {
		shooter.calculate();
		return false;
	}

	@Override
	// When activated, stops the robot
	public void override() {
		this.io.setShooterTopMotor(0.0);
		this.io.setShooterBottomMotor(0.0);

	}

}
