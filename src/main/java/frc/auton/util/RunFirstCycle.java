package frc.auton.util;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.subsystems.Drive;
import frc.subsystems.Hangar;
import frc.subsystems.Intake;
import frc.subsystems.Shooter;




public class RunFirstCycle extends AutonCommand {
    private Drive drive;
    private Intake intake;
    private Shooter shooter;
    private Hangar hanger;
    
    

    public RunFirstCycle() {
        super(RobotComponent.UTIL);
       this.drive = Drive.getInstance();
       this.intake = Intake.getInstance();
       this.shooter = Shooter.getInstance();
       this.hanger = Hangar.getInstance();
    }

    @Override
    public void firstCycle() {
        this.drive.firstCycle();
        this.intake.firstCycle();
        this.shooter.firstCycle();
        this.hanger.firstCycle();

    }

    @Override
    public boolean calculate() {
        return true;
    }

    @Override
    public void override() {

    }

}
