package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.global.SystemsConstants;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

public class Deflector {

    public Servo deflector;

    public Deflector(HardwareMap hardwareMap) {
        deflector = hardwareMap.get(Servo.class, "Deflector");
    }

    public void move(double pose) {
        deflector.setPosition(pose);
    }

    public void moveAtAngleInDegrees(double angle) {
        deflector.setPosition(MathUtils.clamp(-0.033 * angle + 2.15, 0.00, 1.00));
    }
    public void moveAtAngleInRadians(double angle) {
        angle = Math.toDegrees(angle);
        deflector.setPosition(MathUtils.clamp(-0.033 * angle + 2.15, 0.00, 1.00));
    }

    public void init(double startPose) {
        deflector.setDirection(Servo.Direction.FORWARD);
        deflector.setPosition(startPose);
    }

    public double getPose() {
        return deflector.getPosition();
    }
}
