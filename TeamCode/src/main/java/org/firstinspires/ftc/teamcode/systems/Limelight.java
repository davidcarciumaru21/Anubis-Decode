package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight {
    private Limelight3A limelight;
    private double lastTxAngle;
    private double lastTaAngle;
    private boolean hasTarget;

    public Limelight(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public double getYaw(){
        LLResult llResult = limelight.getLatestResult();


        if (llResult != null && llResult.isValid()){
            lastTxAngle = llResult.getTx();
            hasTarget = true;
        } else {
            hasTarget = false;
        }

        return lastTxAngle;
    }

    public double getTargetArea(){
        LLResult llResult = limelight.getLatestResult();


        if (llResult != null && llResult.isValid()){
            lastTaAngle = llResult.getTa();
        } else {
        }

        return lastTaAngle;
    }

    public boolean hasTarget(){
        return hasTarget;
    }
}
