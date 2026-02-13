package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Indexer {

    private DcMotorEx indexer;

    public Indexer(HardwareMap hardwareMap) {
        indexer = hardwareMap.get(DcMotorEx.class, "Indexer");
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void pull() {
        indexer.setPower(1.0);
    }

    public void push() {
        indexer.setPower(-1.0);
    }

    public void off() {
        indexer.setPower(0.0);
    }

}
