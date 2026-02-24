package org.firstinspires.ftc.teamcode.opModes.teleOp.dev.controllerTest;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ControllerSpikeTestTeleOp", group = "Dev")
public class ControllerSpikeTestTeleOp extends OpMode {

    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        panelsTelemetry.addData("Left stick x", gamepad1.left_stick_x);
        panelsTelemetry.addData("Left stick y", gamepad1.left_stick_y);
        panelsTelemetry.addData("Right stick x", gamepad1.right_stick_x);
        panelsTelemetry.update();
    }
}
