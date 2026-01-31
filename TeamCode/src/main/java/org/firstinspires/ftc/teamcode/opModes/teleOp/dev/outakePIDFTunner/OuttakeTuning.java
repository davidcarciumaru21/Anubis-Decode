package org.firstinspires.ftc.teamcode.opModes.teleOp.dev.outakePIDFTunner;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OuttakeTuning", group = "Tuning")
public class OuttakeTuning extends SelectableOpMode {
    public OuttakeTuning() {
        super("Select Outtake Mode", s -> {
            s.add("kV", OuttakeKvTuner::new);
            s.add("kS", OuttakeKsTuner::new);
            s.add("PID", OuttakePITuner::new);
        });
    }
}

