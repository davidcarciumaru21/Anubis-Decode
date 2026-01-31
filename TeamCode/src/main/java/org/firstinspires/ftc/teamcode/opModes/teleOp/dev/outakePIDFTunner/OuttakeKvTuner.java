package org.firstinspires.ftc.teamcode.opModes.teleOp.dev.outakePIDFTunner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.systems.Outtake;

@TeleOp(name = "OuttaleKvTunner", group = "Tuning")
public class OuttakeKvTuner extends OpMode {

    private Outtake outtake;
    private VoltageSensor voltageSensor;

    @Override
    public void init() {
        outtake = new Outtake(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {
        outtake.setPower(0.5);
        telemetry.addData("Kv", voltageSensor.getVoltage() * 0.5 / outtake.getRPM());
        telemetry.update();
    }
}
