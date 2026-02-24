package org.firstinspires.ftc.teamcode.opModes.teleOp.dev.deflectorTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Deflector;

@TeleOp(name = "DeflectorTestTeleOp", group = "dev")
public class DeflectorTestTeleOp extends OpMode {

    private Deflector deflector;
    private double pose;

    @Override
    public void init() {
        deflector = new Deflector(hardwareMap);
    }

    @Override
    public void start() {
        pose = 0.12;
    }

    @Override
    public void loop() {
        deflector.move(pose);

        if (gamepad1.dpadUpWasPressed()) pose += 0.01;
        if (gamepad1.dpadDownWasPressed()) pose -= 0.01;

        telemetry.addData("pose", pose);
        telemetry.update();
    }
}
