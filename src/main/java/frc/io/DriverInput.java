package frc.io;

import frc.util.LogitechF310Gamepad;

public class DriverInput {

	private static DriverInput instance;

	private LogitechF310Gamepad driver;
	private LogitechF310Gamepad operator;
	
	// Creates boolean variables that stores if a certain step/mode was pressed
	private boolean autonIncreaseStepWasPressed = false;
	private boolean autonDecreaseStepWasPressed = false;

	private boolean autonIncreaseModeWasPressed = false;
	private boolean autonDecreaseModeWasPressed = false;

	private boolean autonIncreaseMode10WasPressed = false;
	private boolean autonDecreaseMode10WasPressed = false;

	private DriverInput() {
		this.driver = new LogitechF310Gamepad(0);
		this.operator = new LogitechF310Gamepad(1);
		
	}

	public static DriverInput getInstance() {
		if (instance == null) {
			instance = new DriverInput();
		}
		return instance;
	}

	/*****************************
	 * DRIVER CONTROLS
	 *****************************/

	// DRIVE
	public double getDriverRightX() {
		if(Math.abs(this.driver.getRightX()) < 0.05){
				return 0;
		}
		return this.driver.getRightX();
	}

	public double getDriverLeftY() {
		if(Math.abs(this.driver.getLeftY()) < 0.05){
			return 0;
		}
		return this.driver.getLeftY();
	}

	public double getDriverRightY(){
		if(Math.abs(this.driver.getRightY()) < 0.05){
			return 0;
		}
		return this.driver.getRightY();
	}

	public double getDriverLeftX(){
		if(Math.abs(this.driver.getLeftX()) < 0.05){
			return 0;
		}
		return this.driver.getLeftX();
	}

	public boolean getDriverManualOnButton(){
		return this.driver.getStartButton();
	}

	public boolean getDriverManualOffButton(){
		return this.driver.getBackButton();
	}


	//HANGAR

	public boolean getHangerOutButton(){
		return this.operator.getPOVUp();
	}

	public boolean getHangerInButton(){
		return this.operator.getPOVDown();
	}

	public double getHangerTrigger(){
		return this.operator.getLeftTrigger() - this.operator.getRightTrigger();
	}

	public boolean getHangerConfirmButton(){
		return this.operator.getGreenButton();
	}

	public boolean getHangerResetButton(){
		return this.operator.getRightBumper();
	}

	public boolean getHangerManualButton(){
		return this.operator.getStartButton();
	}

	public boolean getHangerManualArmUpButton(){
		return this.operator.getRedButton();
	}

	public boolean getHangerManualArmDownButton(){
		return this.operator.getGreenButton();
	}


	// INTAKE

	public boolean getIntakeOnButton(){

		return this.driver.getRightBumper();

	}

	public boolean getIntakeReverseButton(){
		
		return this.driver.getLeftBumper();

	}

	public boolean getIntakeUpButton(){
		return this.operator.getLeftTrigger() > 0.1;
	}

	// SHOOTER
	public boolean getShooterFenderButton(){
		return this.operator.getRedButton();
	}

	public boolean getShooterLaunchPadButton(){
		return this.operator.getYellowButton();
	}

	public boolean getShooterOffButton(){
		return this.operator.getGreenButton();
	}

	public boolean getShooterLowGoalButton(){
		return this.operator.getBlueButton();
	}

	public boolean getShooterLimeLockButton(){
		return this.driver.getLeftTrigger() > 0.1;
	}

	public boolean getDriverShootWhileMovingButton(){
		return false;//this.driver.getGreenButton();
	}

	public boolean getDriverSnapToPresetButton(){
		return false;//this.driver.getRedButton();
	}

	public boolean getDriverGetColorSensorOffButton(){
		return this.driver.getBlueButton();
	}

	public boolean getDriverGetColorSensorOnButton(){
		return this.driver.getYellowButton();
	}

	public boolean getShooterShootButton(){
		return false;
	}

	public boolean getShooterForceShootButton(){
		return this.driver.getRightTrigger() > 0.1;
	}




	public boolean getResetGyroButton(){
		return this.driver.getPOVUp();
	}



	// ********************************
	// AUTO SELECTION CONTROLS
	// ********************************

	public boolean getDriverAutoOverrideButtons() {
		return this.driver.getGreenButton();
	}

	public boolean getOperatorAutoOverrideButtons() {
		return false;//this.operator.getGreenButton();
	}

	public boolean getAutonSetDelayButton() {
		return false;// this.driver.getRightTrigger() > 0.2;
	}

	public double getAutonDelayStick() {
		return this.driver.getLeftY();
	}

	public boolean getAutonStepIncrease() {
		// only returns true on rising edge
		boolean result = this.driver.getRightBumper() && !this.autonIncreaseStepWasPressed;
		this.autonIncreaseStepWasPressed = this.driver.getRightBumper();
		return result;

	}

	public boolean getAutonStepDecrease() {
		// only returns true on rising edge
		boolean result = this.driver.getLeftBumper() && !this.autonDecreaseStepWasPressed;
		this.autonDecreaseStepWasPressed = this.driver.getLeftBumper();
		return result;

	}

	public boolean getAutonModeIncrease() {
		// only returns true on rising edge
		boolean result = this.driver.getRedButton() && !this.autonIncreaseModeWasPressed;
		this.autonIncreaseModeWasPressed = this.driver.getRedButton();
		return result;

	}

	public boolean getAutonModeDecrease() {
		// only returns true on rising edge
		boolean result = this.driver.getGreenButton() && !this.autonDecreaseModeWasPressed;
		this.autonDecreaseModeWasPressed = this.driver.getGreenButton();
		return result;

	}

	public boolean getAutonModeIncreaseBy10() {
		// only returns true on rising edge
		boolean result = this.driver.getYellowButton() && !this.autonIncreaseMode10WasPressed;
		this.autonIncreaseMode10WasPressed = this.driver.getYellowButton();
		return result;

	}

	public boolean getAutonModeDecreaseBy10() {
		// only returns true on rising edge
		boolean result = this.driver.getBlueButton() && !this.autonDecreaseMode10WasPressed;
		this.autonDecreaseMode10WasPressed = this.driver.getBlueButton();
		return result;

	}

}