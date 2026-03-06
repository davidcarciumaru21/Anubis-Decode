package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.IMU;

public class Limelight {
    private Limelight3A limelight;
    private IMU imu;
    private double lastTxAngle;
    private double lastX = 1.8;
    private double lastY = 1.8;
    private double lastHeading;
    private double lastTaAngle;
    private boolean hasTarget;

    public Limelight(HardwareMap hardwareMap, int index){
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(index);
        limelight.start();
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
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

    public Pose getPose() {
        LLResult llResult = limelight.getLatestResult();
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());

        if (llResult == null || !llResult.isValid()) return null;

        Pose3D wpiPose = llResult.getBotpose();

        if (llResult.getBotpose().getPosition().z >= 30) return null;

        double xInches = wpiPose.getPosition().x * 39.3701;
        double yInches = wpiPose.getPosition().y * 39.3701;

        double xPedro = xInches + 72;
        double yPedro = Math.abs(yInches) + 72;

        double headingPedro = -Math.toRadians(wpiPose.getOrientation().getYaw());

        return new Pose(xPedro, yPedro, headingPedro);
    }

    public boolean hasTarget() {
        LLResult llResult = limelight.getLatestResult();
        return llResult != null && llResult.isValid();
    }
}
