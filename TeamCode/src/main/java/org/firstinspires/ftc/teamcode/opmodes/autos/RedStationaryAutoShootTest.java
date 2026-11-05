package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/** Place at the half-route's first shooting point; validate one shot without running a path. */
@Config
@Autonomous(name = "Red Fixed Shot Shared Aim v1", group = "Test")
public final class RedStationaryAutoShootTest extends LinearOpMode {
    // Pedro inches, intake/front heading. Physical placement must match this estimate.
    public static double startX = 53.8928;
    public static double startY = 31.5834;
    public static double startHeadingDegrees = 270;
    public static long feedDurationMs = 2_000;
    public static boolean waitForHigh = false;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();
        Follower follower = null;
        Shooter shooter = null;
        Intake intake = null;
        Transit transit = null;
        Vision vision = null;
        try {
            if (!Double.isFinite(startX) || !Double.isFinite(startY)
                    || !Double.isFinite(startHeadingDegrees)) {
                throw new IllegalArgumentException("Fixed shot requires a finite starting pose");
            }
            Pose start = new Pose(startX, startY, Math.toRadians(startHeadingDegrees));
            follower = Constants.create(hardwareMap);
            shooter = new Shooter(hardwareMap);
            intake = new Intake(hardwareMap);
            transit = new Transit(hardwareMap);
            vision = new Vision(hardwareMap, Alliance.RED);
            AutoHiveTarget target = new AutoHiveTarget(vision, CellCluster.RED_OPPOSITE_AUDIENCE);
            GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                    Constants.localizerConfig.name.get());
            follower.setPose(start);
            while (opModeInInit() && !isStopRequested()) {
                follower.update();
                scheduler.run();
                telemetry.addLine("Fixed shot v1: no path; robot will turn to aim after START.");
                telemetry.addData("Place Pedro X/Y/front", "%.4f / %.4f in / %.1f deg",
                        start.x(), start.y(), Math.toDegrees(start.heading()));
                telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            long started = System.nanoTime();
            while (opModeIsActive() && pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                    && (System.nanoTime() - started) / 1_000_000L < 5_000) {
                follower.update();
                scheduler.run();
                telemetry.addData("Waiting for Pinpoint", pinpoint.getDeviceStatus());
                telemetry.update();
                idle();
            }
            if (!opModeIsActive()) return;
            if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                throw new IllegalStateException("Pinpoint did not become READY");
            }
            follower.setPose(start);
            AutoShootCommand shot = new AutoShootCommand(follower, shooter, intake, transit, target,
                    () -> pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY,
                    feedDurationMs, 13_000, waitForHigh);
            scheduler.schedule(shot);
            while (opModeIsActive()) {
                if (scheduler.isScheduled(shot)) {
                    follower.update();
                    if ((System.nanoTime() - started) / 1_000_000L >= 30_000
                            || pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                        scheduler.cancel(shot);
                    } else {
                        scheduler.run();
                    }
                }
                telemetry.addData("Build", "fixed-shot-v1-shared-auto-aim");
                telemetry.addData("Shot", "%s / %s", shot.getResult(), shot.getStatus());
                telemetry.addData("Yaw / position error", "%.1f deg / %.2f in",
                        shot.getYawErrorDegrees(), shot.getPositionErrorInches());
                telemetry.addData("Distance / feed", "%.1f cm / %d ms",
                        shot.getDistanceCm(), shot.getFeedElapsedMs());
                telemetry.addData("Shooter actual / target", "%.0f / %.0f ticks/s",
                        shooter.getRightVelocity(), shooter.getTargetVelocity());
                telemetry.addData("Transit", transit.getFiringState());
                telemetry.update();
                idle();
            }
        } catch (RuntimeException error) {
            RobotLog.ee("RedStationaryAutoShootTest", error, "Fixed shot failed");
            telemetry.addData("Fixed shot error", error.toString());
            telemetry.update();
        } finally {
            scheduler.cancelAll();
            if (transit != null) transit.cancelFeeding();
            if (intake != null) intake.stop();
            if (shooter != null) shooter.stop();
            if (vision != null) vision.stop();
            if (follower != null) {
                follower.stop();
                follower.drivetrain.stop();
            }
            scheduler.reset();
        }
    }
}
