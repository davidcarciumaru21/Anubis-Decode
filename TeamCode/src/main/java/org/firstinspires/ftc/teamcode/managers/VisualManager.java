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

    /*
    public double getTargetRotation(double currentTime, Follower follower, Pose pose) {
        if (limelight.hasTarget()) {

            double yawOffset = 2.5;
            double error = limelight.getYaw() - yawOffset;

            double kP = 0.025;
            double kD = 0.002;

            double dt = currentTime - lastTime;
            if (dt > 0) {
                derivative = (error - lastError) / dt;
            }

            lastError = error;
            lastTime = currentTime;

            double output = -(kP * error + kD * derivative);

            if (Math.abs(error) < 0.1) {
                return 0;
            }

            output = Math.max(-0.7, Math.min(0.7, output));

            return output;

        }
        return 0;
    }

     */

    public Pose getNewPose() {
        return limelight.getPose();
    }
}
