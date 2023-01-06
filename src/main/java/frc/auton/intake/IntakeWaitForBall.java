package frc.auton.intake;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.IO;
import frc.subsystems.Intake;
import frc.subsystems.Intake.IntakeState;

public class IntakeWaitForBall extends AutonCommand {

    private IO io;
    private Intake intake;
	public IntakeWaitForBall(long timeout) {
		super(RobotComponent.INTAKE, timeout);
        this.io = IO.getInstance();
        this.intake = Intake.getInstance();
	}

	@Override
	public boolean calculate() {
        this.intake.calculate();
		if(this.io.getIndexerBallCount() == 2){
            return true;
        }else {
            return false;
        }

	}

	@Override
	public void override() {
		// TODO Auto-generated method stub

	}

	@Override
	public void firstCycle() {
		// TODO Auto-generated method stub
        this.intake.setState(IntakeState.INTAKING_DOWN);

	}

}