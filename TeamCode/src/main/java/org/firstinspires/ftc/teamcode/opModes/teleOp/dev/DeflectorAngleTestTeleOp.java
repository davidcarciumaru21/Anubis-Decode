package org.firstinspires.ftc.teamcode.opModes.teleOp.dev;

import androidx.arch.core.internal.SafeIterableMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Deflector;

@TeleOp(name = "DeflectorAngleTestTeleOp", group = "dev")
public class DeflectorAngleTestTeleOp extends OpMode {

    private Deflector deflector;
    private double angle;

    @Override
    public void init() {
        deflector = new Deflector(hardwareMap);
    }

    @Override
    public void start() {
        angle = 50;
    }

    @Override
    public void loop() {
        deflector.moveAtAngleInDegrees(angle);

        if (gamepad1.dpadUpWasPressed()) angle += 1;
        if (gamepad1.dpadDownWasPressed()) angle -= 1;

        telemetry.addData("angle", angle);
        telemetry.update();
    }
}
