package org.firstinspires.ftc.teamcode.managers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.global.Poses;
import org.firstinspires.ftc.teamcode.systems.Limelight;

public class VisualManager {
    Limelight limelight;

    private double Ta;
    private double distance;
    private double offset = 10;
    private double lastError = 0;
    private double derivative;
    private double lastTime = 0;
    private double rotation;

    public VisualManager(Limelight limelight) {
        this.limelight = limelight;
    }

    public double getDistance() {
        Ta = limelight.getTargetArea();
        distance = -0.0054583 * Ta + 1.1595 + offset;
        return distance;
    }

    public double getTargetRotation(double currentTime, Follower follower, Pose pose) {
        if(pose.getPose() == Poses.blueGoalPose.getPose()){
            limelight.setPipelineSwitch(0);
        }
        else{
            limelight.setPipelineSwitch(0); // RED
        }
        double odoError;
        {
            double dx = pose.getX() - follower.getPose().getX();
            double dy = pose.getY() - follower.getPose().getY();

            double baseHeading = Math.toRadians(90) - Math.atan2(dx, dy);
            double targetHeading = baseHeading;

            double currentHeading = follower.getPose().getHeading();
            odoError = targetHeading - currentHeading;

            while (odoError > Math.PI)  odoError -= 2 * Math.PI;
            while (odoError < -Math.PI) odoError += 2 * Math.PI;
        }

        //rotation = odoError / Math.PI;

        if (limelight.hasTarget()) {

            double error = limelight.getYaw();
            double kP = 0.015;   // tune this
            double dt = currentTime - lastTime;
            if (dt > 0) {
                derivative = (error - lastError) / dt;
            }

            double kD = 0.000000001;

            return -(kP * error + kD * derivative);
        }

        return  rotation;

    }
}
