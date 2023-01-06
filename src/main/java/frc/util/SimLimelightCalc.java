package frc.util;

import frc.imaging.SimLimelight;

public class SimLimelightCalc{

    public SimLimelightCalc instance;
    public SimLimelight limelight;
    
    public SimLimelightCalc(){
        this.limelight = SimLimelight.getInstance(0);
    }

    public SimLimelightCalc getInstance(){
        if(instance == null){
            instance = new SimLimelightCalc();
        }

        return instance;
    }
}