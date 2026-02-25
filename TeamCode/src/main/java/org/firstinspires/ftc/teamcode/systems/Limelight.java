package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight {
    private Limelight3A limelight;
    private double lastTxAngle;
    private double lastX;
    private double lastY;
    private double lastHeading;
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
            lastX = llResult.getTx();
            hasTarget = true;
        } else {
            hasTarget = false;
        }

        return lastTxAngle;
    }

    public double getX(){
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose();
            if(Math.abs(lastY - (botPose.getPosition().x + 1.8)) < 5) {
                lastX = botPose.getPosition().x + 1.8;
            }
            hasTarget = true;

        } else {
            hasTarget = false;
        }

        return lastX;
    }

    public double getY(){
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose();
            if(Math.abs(lastY - (botPose.getPosition().y + 1.8)) < 5) {
                lastY = botPose.getPosition().y + 1.8;
            }
            hasTarget = true;

        } else {
            hasTarget = false;
        }

        return lastY;
    }

    public double getHeading(){
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose();
            if(Math.abs(lastY - botPose.getOrientation().getYaw()) < 5) {
                lastHeading = botPose.getOrientation().getYaw();
            }
            hasTarget = true;
        } else {
            hasTarget = false;
        }

        return lastHeading;
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
