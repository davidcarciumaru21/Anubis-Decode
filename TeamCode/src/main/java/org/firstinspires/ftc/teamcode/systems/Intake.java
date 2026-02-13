package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {

    private DcMotorEx intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void pull() {
        intake.setPower(1.0);
    }
    public boolean isOk(){
        return intake.isOverCurrent();
    }

    public void push() {
        intake.setPower(-1.0);
    }

    public void off() {
        intake.setPower(0.0);
    }

    public double getCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }
}
