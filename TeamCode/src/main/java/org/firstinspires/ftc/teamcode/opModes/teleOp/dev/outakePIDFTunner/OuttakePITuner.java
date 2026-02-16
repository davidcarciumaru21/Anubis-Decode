package org.firstinspires.ftc.teamcode.opModes.teleOp.dev.outakePIDFTunner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Outtake;

@TeleOp(name = "PITunner", group = "Tuning")
public class OuttakePITuner extends OpMode {

    private Outtake outtake;
    private double kP = 0, kI = 0;
    private int power = 0, target = 0;

    private TelemetryManager panelsTelemetry;

    private ElapsedTime timer;

    @Override
    public void init() {
        outtake = new Outtake(hardwareMap);
        outtake.changePI(kP, kI);

        timer = new ElapsedTime();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed()) power += 1;
        if (gamepad1.leftBumperWasPressed()) power -= 1;

        if (gamepad1.dpadUpWasPressed()) kP += Math.pow(10, power);
        if (gamepad1.dpadDownWasPressed()) kP -= Math.pow(10, power);

        if (gamepad1.dpadRightWasPressed()) kI += Math.pow(10, power);
        if (gamepad1.dpadLeftWasPressed()) kI -= Math.pow(10, power);

        outtake.changePI(kP, kI);

        if (gamepad1.aWasPressed()) target = 0;
        if (gamepad1.yWasPressed()) target = 3000;

        outtake.moveFlyWheelAtRPM(target);
        outtake.update(timer.milliseconds());
        timer.reset();

        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("target", target);
        telemetry.addData("current RPM", outtake.getRPM());

        panelsTelemetry.addData("kP", kP);
        panelsTelemetry.addData("kI", kI);
        panelsTelemetry.addData("target", target);
        panelsTelemetry.addData("current RPM", outtake.getRPM());

        panelsTelemetry.update();
        telemetry.update();
    }

}
