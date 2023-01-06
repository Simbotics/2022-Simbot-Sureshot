package frc.auton.intake;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.subsystems.Intake;
import frc.subsystems.Intake.IntakeState;

public class IntakeSetState extends AutonCommand {

    private IO io;
    private Intake intake;
    private Intake.IntakeState intakeState;

    public IntakeSetState(Intake.IntakeState intakeState){
        super(RobotComponent.INTAKE);
        this.io = IO.getInstance();
        this.intake = Intake.getInstance();
        this.intakeState = intakeState;
    }

	@Override
	public void firstCycle() {
		intake.setState(this.intakeState);
	}

	@Override
	// Sets the motor outputs
	public boolean calculate() {
		intake.calculate();
		return false;
	}

	@Override
	// When activated, stops the robot
	public void override() {
		this.io.setIntakeMotor(0.0);
		this.io.setIndexerBottom(0.0);
	}

}