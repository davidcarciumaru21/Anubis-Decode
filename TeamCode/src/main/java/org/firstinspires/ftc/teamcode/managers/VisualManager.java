package org.firstinspires.ftc.teamcode.managers;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.global.AllianceColor;
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

    public double getTargetRotation(double currentTime,
                                    Follower follower,
                                    double distanceToGoal,
                                    AllianceColor allianceColor) {
        double odoError;
        {
            double dx = Poses.blueGoalPose.getX() - follower.getPose().getX();
            double dy = Poses.blueGoalPose.getY() - follower.getPose().getY();

            double baseHeading = Math.toRadians(90) - Math.atan2(dx, dy);
            double targetHeading = baseHeading;

            double currentHeading = follower.getPose().getHeading();
            odoError = targetHeading - currentHeading;

            while (odoError > Math.PI)  odoError -= 2 * Math.PI;
            while (odoError < -Math.PI) odoError += 2 * Math.PI;
        }

        rotation = odoError / Math.PI;

        if (limelight.hasTarget()) {

            double coef = 0.0;
            if (distanceToGoal > 125 && allianceColor == AllianceColor.BLUE) {
                coef = 3.0;
            } else if (distanceToGoal > 125 && allianceColor == AllianceColor.RED) {
                coef = -3.0;
            }

            double error = limelight.getYaw();
            double kP = 0.016;
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
