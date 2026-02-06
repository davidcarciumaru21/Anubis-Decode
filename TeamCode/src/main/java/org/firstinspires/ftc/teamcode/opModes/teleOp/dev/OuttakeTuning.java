package org.firstinspires.ftc.teamcode.opModes.teleOp.dev;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.teleOp.dev.outakePIDFTunner.OuttakeKsTuner;
import org.firstinspires.ftc.teamcode.opModes.teleOp.dev.outakePIDFTunner.OuttakeKvTuner;
import org.firstinspires.ftc.teamcode.opModes.teleOp.dev.outakePIDFTunner.OuttakePITuner;

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

