package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deflector {

    public Servo deflector;

    public Deflector(HardwareMap hardwareMap) {
        deflector = hardwareMap.get(Servo.class, "Deflector");
    }

    public void move(double pose) {
        deflector.setPosition(pose);
    }

    public void init(double startPose) {
        deflector.setDirection(Servo.Direction.FORWARD);
        deflector.setPosition(startPose);
    }
}
