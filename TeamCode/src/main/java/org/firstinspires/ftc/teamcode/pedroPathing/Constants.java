package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11)
            .forwardZeroPowerAcceleration(-40.2098863)
            .lateralZeroPowerAcceleration(-64.75)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,
                    0,
                    0.001,
                    0.03
            ))
            .useSecondaryDrivePIDF(true)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.15,
                    0,
                    0.035,
                    0.005
            ))
            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1,
                    0,
                    0.001,
                    0.03
            ))
            .useSecondaryDrivePIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2,
                    0,
                    0.1,
                    0.005
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.5,
                    0,
                    0.0001,
                    0.6,
                    0.03
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.01,
                    0,
                    0.00001,
                    0.6,
                    0.005
            ))
            .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("FrontLeftMotor")
            .leftRearMotorName("BackLeftMotor")
            .rightFrontMotorName("FrontRightMotor")
            .rightRearMotorName("BackRightMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(-0.0020159185)
            .strafeTicksToInches(-0.0019831135)
            .turnTicksToInches(0.001963445)
            .leftPodY(4.4326377952755905511811023622047)
            .rightPodY(-4.4326377952755905511811023622047)
            .strafePodX(-6.1001574803149606299)
            .leftEncoder_HardwareMapName("Intake")
            .rightEncoder_HardwareMapName("BackLeftMotor")
            .strafeEncoder_HardwareMapName("FrontRightMotor")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}
