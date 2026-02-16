package org.firstinspires.ftc.teamcode.opModes.teleOp.dev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Outtake;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "OuttakeTestTeleOp", group = "Dev")
public class OuttakeTestTeleOp extends OpMode {

    private Outtake outtake;
    private static final double TARGET_RPM = 3000;
    private static final double RPM_THRESHOLD = 2950;

    private TelemetryManager panelsTelemetry;

    private ElapsedTime timer;
    private ElapsedTime totalTimer;

    private boolean reachedSpeed = false;
    private double timeToSpeedMs = 0.0;

    @Override
    public void init() {
        outtake = new Outtake(hardwareMap);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        timer = new ElapsedTime();
        totalTimer = new ElapsedTime();

        reachedSpeed = false;
        timeToSpeedMs = 0.0;
    }

    @Override
    public void loop() {
        outtake.moveFlyWheelAtRPM(TARGET_RPM);

        double rpm = outtake.getRPM();
        double power = outtake.getPower();

        //============================Time to speed=============================
        if (!reachedSpeed && rpm >= RPM_THRESHOLD) {
            timeToSpeedMs = totalTimer.milliseconds();
            reachedSpeed = true;
        }

        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Current RPM", rpm);
        telemetry.addData("Power", power);
        telemetry.addData("timeToSpeed(ms)", timeToSpeedMs);

        panelsTelemetry.addData("target", TARGET_RPM);
        panelsTelemetry.addData("rpm", rpm);
        panelsTelemetry.addData("power", power);
        panelsTelemetry.addData("timeToSpeed(ms)", timeToSpeedMs);

        telemetry.update();
        panelsTelemetry.update();

        outtake.update(timer.milliseconds());
        timer.reset();
    }

    @Override
    public void stop() {
        telemetry.addLine("Start Code #24037");
        telemetry.update();
    }
}