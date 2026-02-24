package org.firstinspires.ftc.teamcode.managers;

import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public double getTargetRotation(double currentTime) {
        if (limelight.hasTarget()) {

            double error = limelight.getYaw();
            double kP = 0.015;   // tune this
            double dt = currentTime - lastTime;
            if (dt > 0) {
                derivative = (error - lastError) / dt;
            }

            double kD = 0.0001;

            return -(kP * error + kD * derivative);

        } else {

            return 0;
        }
    }
}
