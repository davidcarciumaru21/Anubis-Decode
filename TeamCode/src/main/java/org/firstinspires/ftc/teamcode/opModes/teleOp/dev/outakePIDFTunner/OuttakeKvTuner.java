package org.firstinspires.ftc.teamcode.opModes.teleOp.dev.outakePIDFTunner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.systems.Outtake;

@TeleOp(name = "Outtake kV Tuner", group = "Tuning")
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

        double rpm = outtake.getRPM();
        double voltage = voltageSensor.getVoltage();
        double appliedVoltage = 0.5 * voltage;

        double kV = (appliedVoltage - outtake.getKs()) / rpm;

        telemetry.addLine("Outtake kV Tuner");
        telemetry.addData("battery", voltageSensor.getVoltage());
        telemetry.addData("rpm", rpm);
        telemetry.addData("applied voltage", appliedVoltage);
        telemetry.addData("kV (V/RPM)", "%.6f", kV);

        telemetry.update();
    }
}
