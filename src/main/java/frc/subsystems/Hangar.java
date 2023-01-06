package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auton.RobotComponent;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.subsystems.LEDStrip.LEDColourState;
import frc.util.SimPID;

public class Hangar extends SubsystemBase {
    private static Hangar instance;
    private HangarState currentHangarState;
    private LEDColourState desiredLedState = LEDColourState.OFF;
    public enum HangarState{FLOOR,CHECKING_CURRENT,EXTENDING_TO_MIDDLE, RETRACTING_TO_MIDDLE, 
            EXTENDING_TO_HIGH, RETRACTING_SEMI_HIGH, SEMI_HIGH, RETRACTING_TO_HIGH, EXTENDING_TO_TRAVERSAL, RETRACTING_TO_TRAVERSAL, MIDDLE, HIGH, TRAVERSAL,
             DONE_EXTENDING_TO_MID, DONE_EXTENDING_TO_HIGH, DONE_EXTENDING_TO_TRAVERSAL, MANUAL }

    private double hangarOutput;
    private int climbingCycles = 0;
    private boolean hasFlippedEncoders = false;
    private int cyclesCheckingCurrent = 0;
    private boolean invertFlag = false;
    private boolean climberInverted = false;
    private SimPID climbPID;
    private boolean manualArm = false;
    private boolean hasTrippedCurrent = false;


    private Hangar() {
        super();
    }

    public static Hangar getInstance(){
        if (instance == null){
            instance = new Hangar();
        }
        return instance;
    }

    public void setState(HangarState newState){
        this.currentHangarState = newState;
    }

    public HangarState getState(){
        return this.currentHangarState;
    }

    public void firstCycle(){
        this.currentHangarState = HangarState.FLOOR;
        this.resetEncoder();
        this.climbPID = new SimPID(RobotConstants.getClimberHoldPID());
        this.climbPID.setMinMaxOutput(0.00, 0.06);
        this.climbPID.setIRange(5000);
    }

    public void setHangarOutput(double output){
        this.hangarOutput = output;
    }

    public void calculate(){
        SmartDashboard.putString("HANGER STATE", this.currentHangarState.toString());
        switch(this.currentHangarState){
            case FLOOR:
                this.io.setHangar(false);
                this.io.setClimberOutput(0.0);
                this.climbingCycles = 0;
                this.cyclesCheckingCurrent = 0;
                this.io.setClimberCurrentLimit(false, 5, 10, 150);
                this.hasFlippedEncoders = false;
                
                
            break;

            case CHECKING_CURRENT:
                this.io.setClimberCurrentLimit(true, 5, 10, 150);    
                this.io.setClimberOutput(this.hangarOutput*0.2);
                this.io.setClimberRampRate(0.5);
                if(this.io.getHangerPostion() > RobotConstants.HANGER_INVERT_THRESHOLD + 1000.0){
                    this.desiredLedState = LEDColourState.CLIMBER_MANUAL_SET;
                } else if(this.io.getHangerPostion() < -RobotConstants.HANGER_INVERT_THRESHOLD - 1000.0){
                    this.desiredLedState = LEDColourState.CLIMBER_MANUAL_SET;
                } else {
                    this.desiredLedState = LEDColourState.CLIMBER_EXTENDING;
                }
            break;
            case EXTENDING_TO_MIDDLE:
                this.io.setClimberRampRate(0.5);
                if(!this.hasFlippedEncoders){
                    if(this.io.getHangerPostion() > RobotConstants.HANGER_INVERT_THRESHOLD){
                        this.hasFlippedEncoders = true;
                        this.climberInverted = false;
                    } else{
                        this.climberInverted = true;
                    }
                }
                this.io.setClimberCurrentLimit(false, 5, 10, 150);

                if(this.climberInverted){
                    if(this.io.getHangerPostion() > -RobotConstants.HANGER_MID_EXTEND_POSITION){
                        this.io.setClimberOutput(-RobotConstants.HANGER_EXTEND_OUTPUT);
                        this.desiredLedState = LEDColourState.CLIMBER_EXTENDING;
                    } else {
                        this.io.setClimberOutput(0.0);
                        this.io.setHangar(true);
                        this.currentHangarState = HangarState.DONE_EXTENDING_TO_MID;
                        this.desiredLedState = LEDColourState.CLIMBER_DONE_EXTENDING;
                    }    
                }else {
                    if(this.io.getHangerPostion() < RobotConstants.HANGER_MID_EXTEND_POSITION){
                        this.io.setClimberOutput(RobotConstants.HANGER_EXTEND_OUTPUT);
                        this.desiredLedState = LEDColourState.CLIMBER_EXTENDING;
                    } else {
                        this.io.setClimberOutput(0.0);
                        this.io.setHangar(true);
                        this.currentHangarState = HangarState.DONE_EXTENDING_TO_MID;
                        this.desiredLedState = LEDColourState.CLIMBER_DONE_EXTENDING;
                    }    
                }
              
               
            break;
            case RETRACTING_TO_MIDDLE:
                
                if(this.climberInverted){
                    if(this.io.getHangerPostion() < -RobotConstants.HANGER_RETRACT_FULL_POSITION){
                        this.io.setClimberOutput(0.75);
                        this.desiredLedState = LEDColourState.CLIMBER_RETRACTING;
                        this.io.setClimberRampRate(0.5);
                        this.hasTrippedCurrent = false;
                    } else if(this.io.getHangerPostion() >= -RobotConstants.HANGER_RETRACT_FULL_POSITION && !this.hasTrippedCurrent){
                        this.io.setClimberOutput(0.75);
                        this.desiredLedState = LEDColourState.CLIMBER_RETRACTING;
                        this.io.setClimberRampRate(0.5);
                        if(Math.abs(this.io.getClimberSpeed()) <= RobotConstants.HANGER_SPEED_THRESHOLD){
                            this.hasTrippedCurrent = true;
                        }
                    } else {
                        this.io.setClimberRampRate(0.1);
                        if(this.io.getHangerPostion() < -RobotConstants.HANGER_RETRACT_HOLD_POSITION){
                            this.io.setClimberOutput(-RobotConstants.HANGER_HOLD_OUTPUT);
                        } else {
                            this.io.setClimberOutput(0.00);
                        }
                        
                        this.io.setHangar(false);
                        this.desiredLedState = LEDColourState.CLIMBER_HOLDING;
                        if(climbingCycles > 75){ //TODO turn back to 50
                            this.currentHangarState = HangarState.MIDDLE;
                        } else {
                            this.climbingCycles++;
                        }
                        
                    }
                } else {
                    if(this.io.getHangerPostion() > RobotConstants.HANGER_RETRACT_FULL_POSITION){
                        this.io.setClimberOutput(-0.75);
                        this.desiredLedState = LEDColourState.CLIMBER_RETRACTING;
                        this.io.setClimberRampRate(0.5);
                        this.hasTrippedCurrent = false;
                    }else if(this.io.getHangerPostion() <= RobotConstants.HANGER_RETRACT_FULL_POSITION && !this.hasTrippedCurrent){
                        this.io.setClimberOutput(-0.75);
                        this.desiredLedState = LEDColourState.CLIMBER_RETRACTING;
                        this.io.setClimberRampRate(0.5);
                        if(Math.abs(this.io.getClimberSpeed()) <= RobotConstants.HANGER_SPEED_THRESHOLD){
                            this.hasTrippedCurrent = true;
                        }
                    } else {
                        this.io.setClimberRampRate(0.1);
                        if(this.io.getHangerPostion() > RobotConstants.HANGER_RETRACT_HOLD_POSITION){
                            this.io.setClimberOutput(RobotConstants.HANGER_HOLD_OUTPUT);
                        } else {
                            this.io.setClimberOutput(-0.00);
                        }
                        
                        this.io.setHangar(false);
                        this.desiredLedState = LEDColourState.CLIMBER_HOLDING;
                        if(climbingCycles > 75){ //TODO turn back to 50
                            this.currentHangarState = HangarState.MIDDLE;
                        } else {
                            this.climbingCycles++;
                        }
                        
                    }
                }
                
            break;
            case MIDDLE:
            this.io.setClimberOutput(0.0);
            this.climbingCycles = 0;
            this.desiredLedState = LEDColourState.CLIMBER_ON_BAR;
            break;
            case EXTENDING_TO_HIGH:
            this.io.setClimberRampRate(0.5);
            if(this.climberInverted){
                if(this.io.getHangerPostion() > -RobotConstants.HANGER_EXTEND_POSITION){
                    this.io.setClimberOutput(-RobotConstants.HANGER_EXTEND_OUTPUT);
                    this.desiredLedState = LEDColourState.CLIMBER_EXTENDING;
                } else {
                    this.io.setClimberOutput(0.0);
                    this.io.setHangar(true);
                    this.currentHangarState = HangarState.DONE_EXTENDING_TO_HIGH;
                    this.desiredLedState = LEDColourState.CLIMBER_DONE_EXTENDING;
                } 
            } else {
                if(this.io.getHangerPostion() < RobotConstants.HANGER_EXTEND_POSITION){
                    this.io.setClimberOutput(RobotConstants.HANGER_EXTEND_OUTPUT);
                    this.desiredLedState = LEDColourState.CLIMBER_EXTENDING;
                } else {
                    this.io.setClimberOutput(0.0);
                    this.io.setHangar(true);
                    this.currentHangarState = HangarState.DONE_EXTENDING_TO_HIGH;
                    this.desiredLedState = LEDColourState.CLIMBER_DONE_EXTENDING;
                }    
            }
            
            break;
            case RETRACTING_SEMI_HIGH:
                if(this.climberInverted){
                    if(this.io.getHangerPostion() < -RobotConstants.HANGER_RETRACT_SEMI_HIGH_POSITION){
                        this.io.setClimberOutput(-RobotConstants.HANGER_RETRACT_OUTPUT);
                        this.desiredLedState = LEDColourState.CLIMBER_RETRACTING;
                        this.io.setClimberRampRate(0.5);
                    } else {
                        this.io.setClimberOutput(0);
                        this.currentHangarState = HangarState.SEMI_HIGH;
                        
                    
                        
                    }
                } else { 
                    if(this.io.getHangerPostion() > RobotConstants.HANGER_RETRACT_SEMI_HIGH_POSITION){
                        this.io.setClimberOutput(RobotConstants.HANGER_RETRACT_OUTPUT);
                        this.desiredLedState = LEDColourState.CLIMBER_RETRACTING;
                        this.io.setClimberRampRate(0.5);
                    } else {
                        this.io.setClimberOutput(0);
                        this.currentHangarState = HangarState.SEMI_HIGH;
                    
                        
                    } 
                }
                
                break;
                case SEMI_HIGH:
                    if(this.climberInverted){
                        this.io.setClimberOutput(-RobotConstants.HANGER_HOLD_OUTPUT);
                    } else {
                        this.io.setClimberOutput(RobotConstants.HANGER_HOLD_OUTPUT);
                    }
                    this.climbingCycles = 0;
                    this.desiredLedState = LEDColourState.CLIMBER_ON_BAR;
                break;
             case RETRACTING_TO_HIGH:
             if(this.climberInverted){
                if(this.io.getHangerPostion() < -RobotConstants.HANGER_RETRACT_FULL_POSITION){
                    this.io.setClimberOutput(-RobotConstants.HANGER_RETRACT_OUTPUT);
                    this.desiredLedState = LEDColourState.CLIMBER_RETRACTING;
                    this.io.setClimberRampRate(0.5);
                    this.hasTrippedCurrent = false;
                }else if(this.io.getHangerPostion() >= -RobotConstants.HANGER_RETRACT_FULL_POSITION && !this.hasTrippedCurrent){
                    this.io.setClimberOutput(-RobotConstants.HANGER_RETRACT_OUTPUT);
                    this.desiredLedState = LEDColourState.CLIMBER_RETRACTING;
                    this.io.setClimberRampRate(0.5);
                    if(Math.abs(this.io.getClimberSpeed()) <= RobotConstants.HANGER_SPEED_THRESHOLD){
                        this.hasTrippedCurrent = true;
                    }
                } else {
                    this.io.setClimberRampRate(0.1);
                    if(this.io.getHangerPostion() < -RobotConstants.HANGER_RETRACT_HOLD_POSITION){
                        this.io.setClimberOutput(-RobotConstants.HANGER_HOLD_OUTPUT);
                    } else {
                        this.io.setClimberOutput(0.00);
                    }
                    this.io.setHangar(false);
                    this.desiredLedState = LEDColourState.CLIMBER_HOLDING;
                    if(climbingCycles > 75){
                        this.currentHangarState = HangarState.HIGH;
                    } else {
                        this.climbingCycles++;
                    }
                    
                }
             } else {
                if(this.io.getHangerPostion() > RobotConstants.HANGER_RETRACT_FULL_POSITION){
                    this.io.setClimberOutput(RobotConstants.HANGER_RETRACT_OUTPUT);
                    this.desiredLedState = LEDColourState.CLIMBER_RETRACTING;
                    this.io.setClimberRampRate(0.5);
                    this.hasTrippedCurrent = false;
                }else if(this.io.getHangerPostion() <= RobotConstants.HANGER_RETRACT_FULL_POSITION && !this.hasTrippedCurrent){
                    this.io.setClimberOutput(RobotConstants.HANGER_RETRACT_OUTPUT);
                    this.desiredLedState = LEDColourState.CLIMBER_RETRACTING;
                    this.io.setClimberRampRate(0.5);
                    if(Math.abs(this.io.getClimberSpeed()) <= RobotConstants.HANGER_SPEED_THRESHOLD){
                        this.hasTrippedCurrent = true;
                    }
                } else {
                    this.io.setClimberRampRate(0.1);
                    if(this.io.getHangerPostion() > RobotConstants.HANGER_RETRACT_HOLD_POSITION){
                        this.io.setClimberOutput(RobotConstants.HANGER_HOLD_OUTPUT);
                    } else {
                        this.io.setClimberOutput(-0.00);
                    }
                    this.io.setClimberOutput(RobotConstants.HANGER_HOLD_OUTPUT);
                    this.io.setHangar(false);
                    this.desiredLedState = LEDColourState.CLIMBER_HOLDING;
                    if(climbingCycles > 75){
                        this.currentHangarState = HangarState.HIGH;
                    } else {
                        this.climbingCycles++;
                    }
                    
                }
             } 
            break;
            case HIGH:
                this.io.setClimberOutput(0.0);
                this.climbingCycles = 0;
                this.desiredLedState = LEDColourState.CLIMBER_ON_BAR;
                break;
            case EXTENDING_TO_TRAVERSAL:
            this.io.setClimberRampRate(0.5);
            if(this.climberInverted){
                if(this.io.getHangerPostion() > -RobotConstants.HANGER_EXTEND_POSITION){
                    this.io.setClimberOutput(-RobotConstants.HANGER_EXTEND_OUTPUT);
                    this.desiredLedState = LEDColourState.CLIMBER_EXTENDING;
                } else {
                    this.io.setClimberOutput(0.0);
                    this.io.setHangar(true);
                    this.currentHangarState = HangarState.DONE_EXTENDING_TO_TRAVERSAL;
                    this.desiredLedState = LEDColourState.CLIMBER_DONE_EXTENDING;
                }
            } else {
                if(this.io.getHangerPostion() < RobotConstants.HANGER_EXTEND_POSITION){
                    this.io.setClimberOutput(RobotConstants.HANGER_EXTEND_OUTPUT);
                    this.desiredLedState = LEDColourState.CLIMBER_EXTENDING;
                } else {
                    this.io.setClimberOutput(0.0);
                    this.io.setHangar(true);
                    this.currentHangarState = HangarState.DONE_EXTENDING_TO_TRAVERSAL;
                    this.desiredLedState = LEDColourState.CLIMBER_DONE_EXTENDING;
                }
            }
                break;
            case RETRACTING_TO_TRAVERSAL:
            if(this.climberInverted){
                if(this.io.getHangerPostion() < -RobotConstants.HANGER_RETRACT_TRAVERSAL_POSITION){
                    this.io.setClimberOutput(-RobotConstants.HANGER_RETRACT_OUTPUT);
                    this.desiredLedState = LEDColourState.CLIMBER_RETRACTING;
                    this.io.setClimberRampRate(0.5);
                } else {
                    this.io.setClimberOutput(0);
                    this.currentHangarState = HangarState.TRAVERSAL;
                    
                   
                    
                }
            } else { 
                if(this.io.getHangerPostion() > RobotConstants.HANGER_RETRACT_TRAVERSAL_POSITION){
                    this.io.setClimberOutput(RobotConstants.HANGER_RETRACT_OUTPUT);
                    this.desiredLedState = LEDColourState.CLIMBER_RETRACTING;
                    this.io.setClimberRampRate(0.5);
                } else {
                    this.io.setClimberOutput(0);
                    this.currentHangarState = HangarState.TRAVERSAL;
                   
                    
                } 
            }
            
                break;
            case TRAVERSAL:
                this.io.setClimberOutput(0.0);
                this.climbingCycles = 0;
                this.desiredLedState = LEDColourState.CLIMBER_TRAVERSAL;
                break;

            case MANUAL:
                this.climbingCycles = 0;
                this.io.setClimberOutput(this.hangarOutput * 0.5);
                this.io.setClimberCurrentLimit(false, 50, 55, 1000);
                
                this.io.setClimberRampRate(0.2);
                this.desiredLedState = LEDColourState.CLIMBER_MANUAL;
                break;
            default:
                break;

        }
    }



    public LEDColourState getDesiredLedState() {
        return this.desiredLedState;
    }
    public void resetEncoder(){
        this.io.resetClimberEncoder();
    }
    public void disable() {
        this.io.setClimberOutput(0);
    }

    public void setManualArm(boolean up){
        this.manualArm = up;
    }
}
