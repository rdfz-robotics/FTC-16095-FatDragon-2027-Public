package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

import java.util.OptionalDouble;

public class Constants {
    // Replace these blocks with the corresponding AutoTune Java exports after tuning.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(DriveConstants.leftFrontMotorName);
        c.backLeftName.set(DriveConstants.leftBackMotorName);
        c.frontRightName.set(DriveConstants.rightFrontMotorName);
        c.backRightName.set(DriveConstants.rightBackMotorName);
        c.frontLeftDirection.set(DriveConstants.leftFrontMotorDirection);
        c.backLeftDirection.set(DriveConstants.leftBackMotorDirection);
        c.frontRightDirection.set(DriveConstants.rightFrontMotorDirection);
        c.backRightDirection.set(DriveConstants.rightBackMotorDirection);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(DriveConstants.pinpointName);
        c.xPodOffset.set(DriveConstants.pinpointXOffsetMm);
        c.yPodOffset.set(DriveConstants.pinpointYOffsetMm);
        c.offsetUnits.set(DistanceUnit.MM);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodDirection.set(DriveConstants.pinpointForwardPodDirection);
        c.yPodDirection.set(DriveConstants.pinpointStrafePodDirection);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.30078914408329804);
                Controller secondaryTranslationalForward = Controller.proportional(0.11113355150482486);
                Controller primaryTranslationalLateral = Controller.proportional(0.9011460873421373);
                Controller secondaryTranslationalLateral = Controller.proportional(0.33294940020599545);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01911059273741491));
                c.brake.set(Controller.proportionalFeedforward(0.01624400382680267));

                c.headingFeedback.set(Controller.proportional(4.717434192704739));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.08312283414119992, -8.564387987015544E-4));

                c.linearBrakeCoefficients.set(Matrix.diag(0.16871065428885162, 0.07420113929680525));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-0.002247858134087847, 0.001471049425502552));

                c.maxAchievableForwardVelocity.set(54.471127258910975);
                c.maxAchievableStrafeVelocity.set(54.65582602741854);
                c.naturalForwardDeceleration.set(64.98071052759337);
                c.naturalStrafeDeceleration.set(90.19049501060306);
            }
    );

    public static Foresight createAlgorithm() {
        if (foresightConfig == null) {
            throw new IllegalStateException(
                    "Run AutoTune > Foresight Tuner, then paste its Java export into pedro/Constants.java");
        }
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap hardwareMap) {
        Foresight algorithm = createAlgorithm();
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                algorithm);
    }
}
