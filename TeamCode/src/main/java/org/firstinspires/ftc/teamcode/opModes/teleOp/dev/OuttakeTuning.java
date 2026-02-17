package org.firstinspires.ftc.teamcode.opModes.teleOp.dev;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Outtake;

@TeleOp(name = "OuttakeTuning", group = "Tuning")
public class OuttakeTuning extends OpMode {

    private Outtake outtake;
    private Intake intake;
    private VoltageSensor voltageSensor;
    private TelemetryManager panelsTelemetry;

    private enum State {
        KS,
        KV,
        PI
    }

    private State state = State.KS;

    private double ksPower = 0.0;
    private double ksVolts = 0.0;

    private static final double TARGET_POWER_KV = 0.5;
    private double kV = 0.0;

    private double kP = 0.0;
    private double kI = 0.0;
    private int exponent = 0;
    private int targetRPM = 0;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        outtake.changePI(kP, kI);
    }

    @Override
    public void loop() {
        switch (state) {
            case KS:
                runKs();
                break;
            case KV:
                runKv();
                break;
            case PI:
                runPi();
                break;
        }
    }

    private void runKs() {
        if (gamepad1.dpadUpWasPressed()) ksPower += 0.01;
        if (gamepad1.dpadDownWasPressed()) ksPower -= 0.01;
        if (gamepad1.right_bumper) ksPower += 0.001;
        if (gamepad1.left_bumper) ksPower -= 0.001;
        if (gamepad1.y) ksPower = 0.0;

        outtake.setPower(ksPower);
        intake.pull();

        double battery = voltageSensor.getVoltage();
        ksVolts = ksPower * battery;

        display(
                "kS Tuning",
                "Power", ksPower,
                "kS (V)", ksVolts,
                "Battery", battery
        );

        if (gamepad1.xWasPressed()) {
            outtake.changeKs(ksVolts);
            state = State.KV;
        }
    }

    private void runKv() {
        outtake.setPower(TARGET_POWER_KV);

        double rpm = outtake.getRPM();
        double battery = voltageSensor.getVoltage();
        double applied = TARGET_POWER_KV * battery;

        if (rpm > 1) {
            kV = (applied - outtake.getKs()) / rpm;
        }

        display(
                "kV Tuning",
                "RPM", rpm,
                "Applied V", applied,
                "kV (V/RPM)", kV
        );

        if (gamepad1.xWasPressed()) {
            outtake.changeKv(kV);
            state = State.PI;
        }
    }

    private void runPi() {
        if (gamepad1.rightBumperWasPressed()) exponent++;
        if (gamepad1.leftBumperWasPressed()) exponent--;

        double step = Math.pow(10, exponent);

        if (gamepad1.dpadUpWasPressed()) kP += step;
        if (gamepad1.dpadDownWasPressed()) kP -= step;
        if (gamepad1.dpadRightWasPressed()) kI += step;
        if (gamepad1.dpadLeftWasPressed()) kI -= step;

        if (gamepad1.aWasPressed()) targetRPM = 0;
        if (gamepad1.yWasPressed()) targetRPM = 3000;

        outtake.changePI(kP, kI);

        outtake.moveFlyWheelAtRPM(targetRPM);
        outtake.update(timer.seconds());
        timer.reset();

        display(
                "PI Tuning",
                "kP", kP,
                "kI", kI,
                "Step", step,
                "Target", targetRPM,
                "RPM", outtake.getRPM()
        );
    }

    private void display(String title, Object... data) {
        telemetry.addLine(title);
        panelsTelemetry.addLine(title);

        for (int i = 0; i < data.length; i += 2) {
            telemetry.addData(data[i].toString(), data[i + 1]);
            panelsTelemetry.addData(data[i].toString(), data[i + 1]);
        }

        telemetry.update();
        panelsTelemetry.update();
    }
}
