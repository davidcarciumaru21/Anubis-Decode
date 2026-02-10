package org.firstinspires.ftc.teamcode.opModes.teleOp.dev.outakePIDFTunner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Outtake;

@TeleOp(name = "OuttakeKsTuner", group = "Tuning")
public class OuttakeKsTuner extends OpMode {

    private Outtake outtake;
    private Intake intake;
    private VoltageSensor voltageSensor;

    private double power = 0.0;

    @Override
    public void init() {
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed()) {
            power += 0.01;
        }
        if (gamepad1.dpadDownWasPressed()) {
            power -= 0.01;
        }

        if (gamepad1.right_bumper) {
            power += 0.001;
        }
        if (gamepad1.left_bumper) {
            power -= 0.001;
        }

        if (gamepad1.y) {
            power = 0.0;
        }

        outtake.setPower(power);
        intake.pull();

        double voltage = voltageSensor.getVoltage();
        double ksVolts = power * voltage;

        telemetry.addLine("Outtake kS Tuner");
        telemetry.addData("Power", "%.4f", power);
        telemetry.addData("kS (Volts)", "%.4f", ksVolts);
        telemetry.addLine("Dpad = ±0.01 | Bumpers = ±0.001 | Y = Reset");

        telemetry.update();
    }
}
