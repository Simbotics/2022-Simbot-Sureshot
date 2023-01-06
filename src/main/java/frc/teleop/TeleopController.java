package frc.teleop;

import frc.io.DriverInput;
import frc.io.IO;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.DriveConstants;
import frc.subsystems.Drive;
import frc.subsystems.Hangar;
import frc.subsystems.Intake;
import frc.subsystems.LEDStrip;
import frc.subsystems.Shooter;
import frc.subsystems.Drive.DriveState;
import frc.subsystems.Hangar.HangarState;

import frc.subsystems.Intake.IntakeState;
import frc.subsystems.LEDStrip.LEDColourState;
import frc.subsystems.Shooter.HoodState;
import frc.subsystems.Shooter.ShooterState;
import frc.util.SimLib;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopController extends TeleopComponent {

	private static TeleopController instance;
	private DriverInput driverIn;
	private IO io;

	private Intake intake;
	private Shooter shooter;
	private Drive drive;
	private Hangar hangar;
	private LEDStrip ledStrip;

	private boolean driverManual = false;
	private boolean operatorManual = false;
	private SlewRateLimiter xLimiter, yLimiter, turningLimiter;

	private boolean hasArmsUpBeenPressed = false;
	private boolean shooterWasAtSpeed = false;

	private boolean firstCycleShooting = true;

	private boolean colorSensorEnabled = true;

	private LEDColourState ledState = LEDColourState.OFF;

	public static TeleopController getInstance() {
		if (instance == null) {
			instance = new TeleopController();
		}
		return instance;
	}

	private TeleopController() {
		
		this.io = IO.getInstance();
		this.driverIn = DriverInput.getInstance();
		
		this.intake = Intake.getInstance();
		this.shooter = Shooter.getInstance();
		this.drive = Drive.getInstance();
		this.hangar = Hangar.getInstance();
		this.ledStrip = LEDStrip.getInstance();
		this.xLimiter = new SlewRateLimiter(2);
		this.yLimiter = new SlewRateLimiter(2);
		this.turningLimiter = new SlewRateLimiter(2.5);

	}

	@Override
	public void firstCycle() {
		this.intake.firstCycle();
		this.shooter.firstCycle();
		this.drive.firstCycle();
		this.hangar.firstCycle();
		this.intake.setColorSensorEnabled(true);
		this.drive.setColorSensorEnabled(true);
	}

	

	@Override
	public void calculate() {

		if(this.driverIn.getDriverManualOnButton()){
			this.driverManual = true;
		} else if(this.driverIn.getDriverManualOffButton()){
			this.driverManual = false;
		}

		if(this.driverIn.getDriverGetColorSensorOnButton()){
			this.intake.setColorSensorEnabled(true);
			this.drive.setColorSensorEnabled(true);
		} else if(this.driverIn.getDriverGetColorSensorOffButton()){
			this.intake.setColorSensorEnabled(false);
			this.drive.setColorSensorEnabled(false);
		}
		
		// Intake
		if(this.driverIn.getIntakeOnButton()){
			shooterWasAtSpeed = false;
			this.intake.setState(IntakeState.INTAKING_DOWN);
			this.shooter.setState(ShooterState.INTAKING);
		} else if(this.driverIn.getIntakeReverseButton()){
			shooterWasAtSpeed = false;
			if(this.intake.isIntakeUp()){
				this.intake.setState(IntakeState.OUTTAKING_UP);
			} else {
				this.intake.setState(IntakeState.OUTTAKING_DOWN);
			}
			
		} else if(this.driverIn.getShooterForceShootButton()){
			shooterWasAtSpeed = false;
			if(this.shooter.getHoodState() == HoodState.OPEN || this.shooter.getHoodState() == HoodState.CLOSED){
				if(this.intake.isIntakeUp()){
					if(this.shooter.getShooterState() == ShooterState.FENDER){
						this.intake.setState(IntakeState.FENDER_SHOOTING_UP);
					} else {
						this.intake.setState(IntakeState.SHOOTING_UP);
					}
				} else {
					if(this.shooter.getShooterState() == ShooterState.FENDER){
						this.intake.setState(IntakeState.FENDER_SHOOTING_DOWN);
					} else {
						this.intake.setState(IntakeState.SHOOTING_DOWN);
					}
					
				}
			} else {
				if(this.intake.isIntakeUp()){
					this.intake.setState(IntakeState.OFF_UP);
				} else {
					this.intake.setState(IntakeState.OFF_DOWN);
				}
			}
		} else if(this.driverIn.getShooterShootButton()){
			if(this.shooter.getHoodState() == HoodState.OPEN || this.shooter.getHoodState() == HoodState.CLOSED){
				if(this.shooter.isShooterAtSpeed() || this.shooterWasAtSpeed){
					this.shooterWasAtSpeed = true;
					if(this.shooter.getShooterState() == ShooterState.LIMELIGHT_SHOOTING){
						if(this.drive.isAimed() && this.drive.isStill()){
							if(this.io.getIndexerBallCount() == 2){ // go into 2 ball shooting state
								if(this.intake.isIntakeUp()){
									this.intake.setState(IntakeState.SHOOTING_UP_TWO_BALLS);
								} else {
									this.intake.setState(IntakeState.SHOOTING_DOWN_TWO_BALLS);
								}
							} else {
								if(this.intake.getState() != IntakeState.SHOOTING_UP_TWO_BALLS && this.intake.getState() != IntakeState.SHOOTING_DOWN_TWO_BALLS){
									if(this.intake.isIntakeUp()){
										this.intake.setState(IntakeState.SHOOTING_UP);
									} else {
										this.intake.setState(IntakeState.SHOOTING_DOWN);
									}
								}
								
							}
							
						} else {
							this.shooterWasAtSpeed = false;
							if(this.intake.isIntakeUp()){
								this.intake.setState(IntakeState.OFF_UP);
							} else {
								this.intake.setState(IntakeState.OFF_DOWN);
							}
						}
					} else { // not using the limelight
						if(this.io.getIndexerBallCount()== 2){ // go into 2 ball shooting state
							if(this.intake.isIntakeUp()){
								this.intake.setState(IntakeState.SHOOTING_UP_TWO_BALLS);
							} else {
								this.intake.setState(IntakeState.SHOOTING_DOWN_TWO_BALLS);
							}
						} else {
							if(this.intake.getState() != IntakeState.SHOOTING_UP_TWO_BALLS && this.intake.getState() != IntakeState.SHOOTING_DOWN_TWO_BALLS){
								if(this.intake.isIntakeUp()){
									this.intake.setState(IntakeState.SHOOTING_UP);
								} else {
									this.intake.setState(IntakeState.SHOOTING_DOWN);
								}
							}
							
						}
					}
				} else { // not at correct RPM
					if(this.intake.isIntakeUp()){
						this.intake.setState(IntakeState.OFF_UP);
					} else {
						this.intake.setState(IntakeState.OFF_DOWN);
					}
				}
				
			} else { // HOOD NOT SET 
				shooterWasAtSpeed = false;
				if(this.intake.isIntakeUp()){
					this.intake.setState(IntakeState.OFF_UP);
				} else {
					this.intake.setState(IntakeState.OFF_DOWN);
				}
			}
		} else { // NOT TRYING TO SHOOT INTAKE OR OUTTAKE
			shooterWasAtSpeed = false;
			if(this.intake.isIntakeUp()){
				this.intake.setState(IntakeState.OFF_UP);
			} else {
				this.intake.setState(IntakeState.OFF_DOWN);
			}
		}


		if(this.driverIn.getShooterForceShootButton() && this.firstCycleShooting){
			System.out.println("Shooter State: " + this.shooter.getShooterState().toString());
			if(this.shooter.getShooterState() == ShooterState.LIMELIGHT_SHOOTING){
				double distance = this.io.getVisionDistanceFeet();
                System.out.println("LIMELIGHT DISTANCE: " + distance);
                System.out.println("LIMELIGHT TOP RPM: " + this.shooter.shooterTopRPMBasedOnDistance(distance));
                System.out.println("LIMELIGHT BOTTOM RPM: " + this.shooter.shooterBottomRPMBasedOnDistance(distance));
			}
			this.firstCycleShooting = false;
		} else if(!this.driverIn.getShooterForceShootButton() && !this.firstCycleShooting) {
			this.firstCycleShooting = true;
		}

		if(this.driverIn.getIntakeUpButton()){
			this.intake.setState(IntakeState.OFF_UP);
			
		}
		
		// Shooter
		if(this.driverIn.getShooterLimeLockButton()){
			this.shooter.setState(ShooterState.LIMELIGHT_SHOOTING);
			this.intake.setCyclesToWaitBetweenShots(10);
		} else if(this.driverIn.getShooterFenderButton() && this.hangar.getState() != HangarState.MANUAL){
			this.shooter.setState(ShooterState.FENDER);
			this.intake.setCyclesToWaitBetweenShots(50);
		} else if(this.driverIn.getShooterLaunchPadButton() && this.hangar.getState() != HangarState.MANUAL){
			this.intake.setCyclesToWaitBetweenShots(10);
			this.shooter.setState(ShooterState.LAUNCH_PAD);
		} else if(this.driverIn.getShooterLowGoalButton() && this.hangar.getState() != HangarState.MANUAL){
			this.intake.setCyclesToWaitBetweenShots(10);
			this.shooter.setState(ShooterState.LOW_GOAL);
		}

		if(this.driverIn.getShooterOffButton() && this.hangar.getState() != HangarState.MANUAL){
			this.shooter.setState(ShooterState.OFF);
		}

		
		

		

		// Drive
		double leftY = SimLib.squareMaintainSign(this.driverIn.getDriverLeftY());
		double leftX = SimLib.squareMaintainSign(this.driverIn.getDriverLeftX());
		double rightX = SimLib.squareMaintainSign(this.driverIn.getDriverRightX());

		// double leftY = this.driverIn.getDriverLeftY();
		// double leftX = this.driverIn.getDriverLeftX();
		// double rightY = this.driverIn.getDriverRightY();

		

		leftX = this.xLimiter.calculate(leftX);
		leftY = this.yLimiter.calculate(leftY);
		rightX = this.turningLimiter.calculate(rightX);
		

		


		if(this.driverIn.getShooterLimeLockButton()){
			this.drive.setState(DriveState.LIMELIGHT_GOAL_LOCK);
		} else if(this.driverIn.getDriverSnapToPresetButton()){
			this.drive.setState(DriveState.PRESET_LOCK);
			if(this.hangar.getState() != HangarState.FLOOR){ // we are trying to climb
				this.drive.setTargetAngle(RobotConstants.DRIVE_CLIMB_ANGLE);
			} else if(this.shooter.getShooterState() == ShooterState.LOW_GOAL || this.shooter.getShooterState() == ShooterState.FENDER){ // lock to one of the faces of the hub
				double boundedAngle = MathUtil.inputModulus(this.io.getHeading(), -180, 180);
				if(boundedAngle >= -24 && boundedAngle <= 66){
					this.drive.setTargetAngle(RobotConstants.DRIVE_FENDER_FRONT_LEFT);
				} else if(boundedAngle >= 66 && boundedAngle <= 156){
					this.drive.setTargetAngle(RobotConstants.DRIVE_FENDER_BACK_LEFT);
				} else if(boundedAngle <= -24 && boundedAngle >= -114){
					this.drive.setTargetAngle(RobotConstants.DRIVE_FENDER_FRONT_RIGHT);
				} else if(boundedAngle >= 156 || boundedAngle <= -114){
					this.drive.setTargetAngle(RobotConstants.DRIVE_FENDER_BACK_RIGHT);
				}
			} else if(this.shooter.getShooterState() == ShooterState.LAUNCH_PAD){
				double boundedAngle = MathUtil.inputModulus(this.io.getHeading(), -180, 180);
				if(boundedAngle >= -33 && boundedAngle <= 57 ){
					this.drive.setTargetAngle(RobotConstants.DRIVE_LAUNCHPAD_ANGLE);
				} else if(boundedAngle <= -33){
					this.drive.setTargetAngle(RobotConstants.DRIVE_RIGHT_WALL_ANGLE);
				} else if(boundedAngle >= 57){
					this.drive.setTargetAngle(RobotConstants.DRIVE_LEFT_WALL_ANGLE);
				}
			}
		} else if(this.driverIn.getDriverShootWhileMovingButton()) {
			this.drive.setState(DriveState.DRIVE_BY);
			this.shooter.setState(ShooterState.LIMELIGHT_MOVING);
		} else {
			if(this.driverManual){
				this.drive.setState(DriveState.MANUAL);
			} else {
				this.drive.setState(DriveState.OUTPUT);
			}
			
		}
		

		if(this.intake.getState() == IntakeState.INTAKING_DOWN || this.intake.getState() == IntakeState.INTAKING_UP){
			leftX *= 0.85;
			leftY *= 0.85;
			rightX *= 0.75;

		}
		
		this.drive.setOutput(leftY, leftX, rightX);


		//Climber

		
			if(this.driverIn.getHangerInButton()){
				switch(this.hangar.getState()){
					case SEMI_HIGH:
						this.hangar.setState(HangarState.RETRACTING_TO_HIGH);
					break;

					case DONE_EXTENDING_TO_MID:
						this.hangar.setState(HangarState.RETRACTING_TO_MIDDLE);
					break;

					case DONE_EXTENDING_TO_TRAVERSAL:
						this.hangar.setState(HangarState.RETRACTING_TO_TRAVERSAL);
					break;
					case DONE_EXTENDING_TO_HIGH:
						this.hangar.setState(HangarState.RETRACTING_SEMI_HIGH);
					default:
						break;
				}
				
			} else if(this.driverIn.getHangerOutButton()){
				switch(this.hangar.getState()){
					case FLOOR:
						this.hangar.setState(HangarState.CHECKING_CURRENT);
						break;
					case HIGH:
						this.hangar.setState(HangarState.EXTENDING_TO_TRAVERSAL);
						break;
					case MIDDLE:
						this.hangar.setState(HangarState.EXTENDING_TO_HIGH);
						break;
					default:
						break;
				}
			} else if(this.driverIn.getHangerResetButton()){
				this.hangar.setState(HangarState.FLOOR);
				this.io.resetClimberEncoder();
			} else if(this.driverIn.getHangerManualButton()){
				this.hangar.setState(HangarState.MANUAL);
			} else if(this.driverIn.getHangerConfirmButton() && this.hangar.getState() == HangarState.CHECKING_CURRENT){
				this.hangar.setState(HangarState.EXTENDING_TO_MIDDLE);
			}
		
		
		this.hangar.setHangarOutput(this.driverIn.getHangerTrigger());
		
		if(this.hangar.getState() == HangarState.MANUAL){
			if(this.driverIn.getHangerManualArmUpButton()){
				
				this.io.setHangar(true);
			} else if(this.driverIn.getHangerManualArmDownButton()){
				
				this.io.setHangar(false);
			}
		}
		
		
		if(this.driverIn.getResetGyroButton()){
			this.drive.resetGyro();
		}


		////////////////////////////
		///////// LED CODE /////////
		////////////////////////////

		if(this.hangar.getState() == HangarState.FLOOR){
			if(this.intake.getState() == IntakeState.OFF_DOWN || this.intake.getState() == IntakeState.OFF_UP){
				if(this.drive.getState() == DriveState.LIMELIGHT_GOAL_LOCK){ // aiming at goal
					if(this.drive.isAimed() && this.io.getVisionDistanceFeet() > 10.0){ // already aimed
						this.ledState = this.shooter.getDesiredLedState();
					} else {
						this.ledState = this.drive.getDesiredLedState();
					}
				} else { // not using limelight check shooter
					if(this.shooter.getShooterState() == ShooterState.OFF || this.shooter.getShooterState() == ShooterState.INTAKING){
						this.ledState = this.intake.getDesiredLedState();
					} else {
						this.ledState = this.shooter.getDesiredLedState();
					}
				}
			} else {
				this.ledState = this.intake.getDesiredLedState();
			}
		} else {
			this.ledState = this.hangar.getDesiredLedState();
		}

		SmartDashboard.putString("LED STATE", this.ledState.toString());
		SmartDashboard.putBoolean("COLOR SENSOR ENABLED: ", this.colorSensorEnabled);

		if(this.driverIn.getDriverGetColorSensorOnButton()){
			this.ledStrip.setLed(LEDColourState.CLIMBER_MANUAL);
		} else if(this.driverIn.getDriverGetColorSensorOffButton()){
			this.ledStrip.setLed(LEDColourState.SHOOTER_NOT_AT_RPM);
		} else if(this.driverIn.getDriverManualOnButton()){
			this.ledStrip.setLed(LEDColourState.VISION_AIMED);
		} else if(this.driverIn.getDriverManualOffButton()){
			this.ledStrip.setLed(LEDColourState.SHOOTER_NOT_AT_RPM);
		} else {
			this.ledStrip.setLed(this.ledState);
		}
		
		
		this.drive.calculate();
		this.intake.calculate();
		this.shooter.calculate();
		this.hangar.calculate();
		

	}

	@Override
	public void disable() {
		this.drive.disable();
		this.intake.disable();
		this.shooter.disable();
		this.hangar.disable();
	}
}
