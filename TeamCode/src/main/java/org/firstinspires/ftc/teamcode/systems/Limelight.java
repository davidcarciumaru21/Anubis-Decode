package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight {
    private Limelight3A limelight;
    private double lastTxAngle;
    private double lastX = 1.8;
    private double lastY = 1.8;
    private double lastHeading;
    private double lastTaAngle;
    private boolean hasTarget;

    public Limelight(HardwareMap hardwareMap, int index){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(index);
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

    public double getX(){
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose();
            if(Math.abs(lastX - (botPose.getPosition().x + 1.8)) < 0.08) {
                if((Math.abs(botPose.getPosition().x + 1.8)*100) < 380) {
                    lastX = botPose.getPosition().x + 1.8;
                }
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
            if(Math.abs(lastY - (botPose.getPosition().y + 1.8)) < 0.08) {
                if((Math.abs(botPose.getPosition().y + 1.8)*100) < 380) {
                    lastY = botPose.getPosition().y + 1.8;
                }
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
            lastHeading = botPose.getOrientation().getYaw();

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
