package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

import java.util.Locale;
import java.util.Map;

/** Test 2: shoot once, travel to the next waypoint, stop for 3 s, cross the field. */
@Config
@Autonomous(name = "Red Half Two Tip Test 2 v6", group = "Test")
public final class RedHalfTwoTipTest extends LinearOpMode {
    private enum Stage { INIT, PATH1, SHOOT, PATH2, DWELL, PATH3, DONE }

    private static final String BUILD_TAG = "red-half-two-tip-test2-v6-exact-route";
    private static final long AUTO_TIMEOUT_MS = 30_000;
    private static final long PINPOINT_WAIT_MS = 5_000;
    private static final long START_POSE_TIMEOUT_MS = 1_500;
    private static final long SHOOT_TIMEOUT_MS = 13_000;
    private static final long DWELL_MS = 3_000;
    private static final long STABLE_MS = 200;
    private static final long TELEMETRY_INTERVAL_MS = 100;
    private static final double ARRIVAL_TOLERANCE_INCHES = 2.5;
    private static final double PATH1_LATERAL_TOLERANCE_INCHES = 3.0;
    public static long feedDurationMs = 2_000;
    private static final double PATH_SPEED_FRACTION = 0.35;
    private static final double ROBOT_HALF_WIDTH_INCHES = 423.0 / (2.0 * 25.4);
    private static final double ROBOT_HALF_LENGTH_INCHES = 368.0 / (2.0 * 25.4);
    private static final double MIN_LEFT_CLEARANCE_INCHES = 0.5;
    private static final double LEFT_CLEARANCE_WARNING_INCHES = 4.0;
    private static String previousRun = "No previous test in this Robot Controller session";

    private final PoseFactory poses = PoseFactory.degrees();
    // Exact Visualizer export, in Pedro inches and degrees. Telemetry also
    // reports field-center coordinates by subtracting 72 from X and Y.
    private final Pose start = poses.of(53.9942, 7.9225, 90);
    private final Pose firstShot = poses.of(53.8928, 31.5834, 270);
    private final Pose point2Start = poses.of(53.8928, 31.5834, 270);
    private final Pose dwellPoint = poses.of(15.1483, 11.4183, 270);
    private final Pose point3Start = poses.of(15.1483, 11.4183, 270);
    private final Pose oppositePoint = poses.of(53.8397, 118.2516, 90);
    private final Pose crossingControl = poses.of(14.6931, 95.546, 0);

    private CommandScheduler scheduler;
    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Transit transit;
    private GoBildaPinpointDriver pinpoint;
    private ShootCommand shootCommand;
    private Pose holdPose;
    private Pose legStartPose;
    private Pose targetPose;
    private Stage stage = Stage.INIT;
    private String status = "Place at Pedro (53.9942, 7.9225), intake heading 90 deg";
    private long autoStartedNanos;
    private long stageStartedNanos;
    private long stableSinceNanos = -1;
    private long previousLoopNanos;
    private double maxLoopMs;
    private String firstDriveCommand;
    private long lastTelemetryNanos;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        scheduler = CommandScheduler.getInstance();
        try {
            if (Constants.foresightConfig == null) {
                throw new IllegalStateException("Foresight config missing; deploy tuned Constants.java");
            }
            prepareDriveMotors();
            follower = Constants.create(hardwareMap);
            shooter = new Shooter(hardwareMap, ShooterConstants.leftShooterDirection.inverted(),
                    ShooterConstants.rightShooterDirection.inverted());
            // This test only feeds preloads. Avoid polling the faulty cds sensor.
            intake = new Intake(hardwareMap, false);
            transit = new Transit(hardwareMap);
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
            follower.setPose(start);
            while (opModeInInit() && !isStopRequested()) {
                follower.update();
                scheduler.run();
                showTelemetry();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            autoStartedNanos = System.nanoTime();
            while (opModeIsActive() && !pinpointReady()
                    && elapsedMs(autoStartedNanos) < PINPOINT_WAIT_MS) {
                follower.update();
                scheduler.run();
                status = "Waiting for Pinpoint: " + pinpoint.getDeviceStatus();
                showTelemetry();
                idle();
            }
            if (isStopRequested()) return;
            if (!pinpointReady()) {
                finish("Pinpoint not READY: " + pinpoint.getDeviceStatus());
            } else if (confirmStartPose()) {
                follow(path1(), Stage.PATH1, "First path to shooting waypoint");
            }
            while (opModeIsActive()) {
                if (stage != Stage.DONE) {
                    long now = System.nanoTime();
                    if (previousLoopNanos != 0) {
                        maxLoopMs = Math.max(maxLoopMs, (now - previousLoopNanos) / 1_000_000.0);
                    }
                    previousLoopNanos = now;
                    follower.update();
                    if (stage == Stage.PATH1 && firstDriveCommand == null) recordFirstDriveCommand();
                    if (!pinpointReady() || !finitePose(follower.pose())) {
                        finish("Stopped: invalid pose or Pinpoint " + pinpoint.getDeviceStatus());
                    } else if (elapsedMs(autoStartedNanos) >= AUTO_TIMEOUT_MS) {
                        finish("30-second test timeout");
                    } else if (checkTravelDirection()) {
                        scheduler.run();
                        advance();
                    }
                }
                showTelemetry();
                idle();
            }
        } catch (RuntimeException error) {
            status = "Error during " + stage + ": " + error;
            stage = Stage.DONE;
            RobotLog.ee("RedHalfTwoTipTest", error, "Autonomous failed");
            stopAll();
            while (!isStopRequested() && (opModeInInit() || opModeIsActive())) {
                telemetry.addData("Build", BUILD_TAG);
                telemetry.addData("Auto error", error.toString());
                telemetry.update();
                idle();
            }
        } finally {
            scheduler.cancelAll();
            stopAll();
            scheduler.reset();
            recordRunSummary();
        }
    }

    private void prepareDriveMotors() {
        String[] motorNames = {
                Constants.drivetrainConfig.frontLeftName.get(),
                Constants.drivetrainConfig.frontRightName.get(),
                Constants.drivetrainConfig.backLeftName.get(),
                Constants.drivetrainConfig.backRightName.get()
        };
        for (String name : motorNames) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
            motor.setPower(0.0);
            // Pedro controls power itself. Clear any encoder-control mode left by a test.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private boolean confirmStartPose() {
        stopFollower();
        follower.setPose(start);
        stableSinceNanos = -1;
        long startedNanos = System.nanoTime();
        status = "Checking start pose readback; keep robot still";
        while (opModeIsActive() && elapsedMs(startedNanos) < START_POSE_TIMEOUT_MS) {
            follower.update();
            scheduler.run();
            if (pinpointReady() && finitePose(follower.pose()) && holdStable(start, 0.5)) {
                return true;
            }
            if (!pinpointReady() || !finitePose(follower.pose())) stableSinceNanos = -1;
            showTelemetry();
            idle();
        }
        finish("Start pose readback failed; expected 53.9942 / 7.9225 / 90 deg");
        return false;
    }

    private void advance() {
        switch (stage) {
            case PATH1:
                if (pathArrived(firstShot, 8_000)) beginShot();
                break;
            case SHOOT:
                advanceShot();
                break;
            case PATH2:
                if (pathArrived(dwellPoint, 8_000)) {
                    stopFollower();
                    enter(Stage.DWELL, "Stopped at waypoint for 3 seconds");
                }
                break;
            case DWELL:
                if (elapsedMs(stageStartedNanos) >= DWELL_MS) {
                    follow(path3(), Stage.PATH3, "Crossing to opposite side; mechanisms stopped");
                }
                break;
            case PATH3:
                if (pathArrived(oppositePoint, 12_000)) finish("Test complete");
                break;
            case INIT:
            case DONE:
                break;
        }
    }

    private void beginShot() {
        // Heading describes the intake/front. At 270 deg the rear shooter faces 90 deg.
        // Hold the exported waypoint throughout firing to preserve the requested route.
        holdPose = firstShot;
        targetPose = holdPose;
        follower.hold(holdPose);
        enter(Stage.SHOOT, "Holding exported waypoint at 270 deg and firing preloads");
        shootCommand = new ShootCommand(shooter, intake, transit,
                this::shotDistanceCm, () -> holdStable(holdPose, 1.5),
                feedDurationMs, SHOOT_TIMEOUT_MS);
        scheduler.schedule(shootCommand);
    }

    private void advanceShot() {
        if (scheduler.isScheduled(shootCommand)) {
            Transit.FiringState firing = shootCommand.getFiringState();
            if ((firing == Transit.FiringState.REVERSING || firing == Transit.FiringState.FEEDING)
                    && !atPose(holdPose, 1.5)) {
                finish("Aim drifted during shot");
            }
            return;
        }
        if (shootCommand.getResult() != ShootCommand.Result.SUCCEEDED) {
            finish("ShootCommand ended: " + shootCommand.getResult());
            return;
        }
        if (!atPose(firstShot, 1.5)) {
            finish("Pose drifted at shot completion");
            return;
        }
        follow(path2(), Stage.PATH2, "Second path; mechanisms stopped");
    }

    private boolean pathArrived(Pose target, long timeoutMs) {
        // isBusy() may become false on Pedro's endpoint timeout. Keep updating
        // its endpoint hold until measured position and heading settle as well.
        if (!follower.isBusy() && holdStable(target, ARRIVAL_TOLERANCE_INCHES)) return true;
        if (follower.isBusy()) stableSinceNanos = -1;
        if (elapsedMs(stageStartedNanos) >= timeoutMs) {
            finish("Path timeout; endpoint error " + Math.round(distanceTo(target) * 10.0) / 10.0 + " in");
        }
        return false;
    }

    private boolean holdStable(Pose target, double positionTolerance) {
        if (!atPose(target, positionTolerance)) {
            stableSinceNanos = -1;
            return false;
        }
        if (stableSinceNanos < 0) stableSinceNanos = System.nanoTime();
        return elapsedMs(stableSinceNanos) >= STABLE_MS;
    }

    private boolean atPose(Pose target, double positionTolerance) {
        return distanceTo(target) <= positionTolerance
                && Math.abs(Math.toDegrees(Math.IEEEremainder(
                follower.pose().heading() - target.heading(), 2.0 * Math.PI))) <= 3.0;
    }

    private double distanceTo(Pose target) {
        return Math.hypot(follower.pose().x() - target.x(), follower.pose().y() - target.y());
    }

    private double shotDistanceCm() {
        double[] tag = VisionConstants.tagClusterCenterFieldXYInches[
                CellCluster.RED_OPPOSITE_AUDIENCE.getTuningIndex()];
        return DistanceUnit.INCH.toCm(Math.hypot(tag[0] - (holdPose.x() - 72.0),
                tag[1] - (holdPose.y() - 72.0)));
    }

    private void follow(Path path, Stage next, String message) {
        legStartPose = follower.pose();
        targetPose = path.endPose();
        follower.follow(path.with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED_FRACTION)));
        enter(next, message);
    }

    private boolean checkTravelDirection() {
        if (leftWallClearanceInches() < MIN_LEFT_CLEARANCE_INCHES) {
            finish("Left-wall clearance below 0.5 in; check actual position and odometry");
            return false;
        }
        if (legStartPose == null) return true;
        double dx = follower.pose().x() - legStartPose.x();
        double dy = follower.pose().y() - legStartPose.y();
        if (stage == Stage.PATH1) {
            if (dy < -2.0) {
                finish("PATH1 moved backward; check placement/localizer");
                return false;
            }
            double lateralError = path1LateralError();
            // Rotation was reported as translation during the physical test.
            // Stop on excessive deviation until the localizer is verified.
            if (lateralError > PATH1_LATERAL_TOLERANCE_INCHES) {
                finish(String.format(Locale.US, "PATH1 lateral deviation %.2f in; run Pinpoint Pose Test",
                        lateralError));
                return false;
            }
        }
        if (stage == Stage.PATH2 && (dx > 2.0 || dy > 2.0)) {
            finish("PATH2 wrong movement: expect -X and -Y; check localizer/drivetrain");
            return false;
        }
        return true;
    }

    private double path1LateralError() {
        double pathX = firstShot.x() - start.x();
        double pathY = firstShot.y() - start.y();
        Pose actual = follower.pose();
        return Math.abs(pathY * (actual.x() - start.x()) - pathX * (actual.y() - start.y()))
                / Math.hypot(pathX, pathY);
    }

    private double leftWallClearanceInches() {
        double heading = follower.pose().heading();
        double extentX = Math.abs(Math.sin(heading)) * ROBOT_HALF_WIDTH_INCHES
                + Math.abs(Math.cos(heading)) * ROBOT_HALF_LENGTH_INCHES;
        return follower.pose().x() - extentX;
    }

    private void recordRunSummary() {
        Pose p = follower == null ? null : follower.pose();
        String poseText = finitePose(p) ? String.format(Locale.US, "Pedro %.2f, %.2f, %.1f deg",
                p.x(), p.y(), Math.toDegrees(p.heading())) : "pose unavailable";
        previousRun = stage + " | " + status + " | " + poseText
                + " | first drive " + firstDriveCommand
                + String.format(Locale.US, " | max loop %.0f ms", maxLoopMs);
        RobotLog.ii("RedHalfTwoTipTest", previousRun);
    }

    private void recordFirstDriveCommand() {
        Map<String, Object> drive = follower.drivetrain.debug();
        firstDriveCommand = String.format(Locale.US, "F/S/T %.3f / %.3f / %.3f",
                drive.get("forward"), drive.get("strafe"), drive.get("turn"));
        RobotLog.ii("RedHalfTwoTipTest", "First drive " + drive);
    }

    private void enter(Stage next, String message) {
        stage = next;
        status = message;
        stageStartedNanos = System.nanoTime();
        stableSinceNanos = -1;
        lastTelemetryNanos = 0;
        RobotLog.ii("RedHalfTwoTipTest", next + " | " + message);
    }

    private void finish(String message) {
        stopAll();
        enter(Stage.DONE, message);
    }

    private void stopAll() {
        if (scheduler != null && shootCommand != null && scheduler.isScheduled(shootCommand)) {
            scheduler.cancel(shootCommand);
        }
        if (transit != null) transit.cancelFeeding();
        if (intake != null) intake.stop();
        if (shooter != null) shooter.stop();
        stopFollower();
    }

    private void stopFollower() {
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop();
        }
    }

    private boolean pinpointReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    private static boolean finitePose(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private static long elapsedMs(long startedNanos) {
        return (System.nanoTime() - startedNanos) / 1_000_000L;
    }

    public Path path1() {
        return line(start, firstShot).linear(start, firstShot);
    }

    public Path path2() {
        return line(point2Start, dwellPoint).linear(point2Start, dwellPoint);
    }

    public Path path3() {
        return curve(point3Start, crossingControl, oppositePoint).linear(point3Start, oppositePoint);
    }

    private void showTelemetry() {
        if (lastTelemetryNanos != 0 && elapsedMs(lastTelemetryNanos) < TELEMETRY_INTERVAL_MS) return;
        lastTelemetryNanos = System.nanoTime();
        telemetry.addData("Build", BUILD_TAG);
        if (stage == Stage.INIT) telemetry.addData("Previous run", previousRun);
        telemetry.addData("Stage", stage);
        telemetry.addData("Status", status);
        telemetry.addData("Ball counting", "Disabled for preload-only test");
        telemetry.addData("Max loop", "%.0f ms", maxLoopMs);
        if (firstDriveCommand != null) telemetry.addData("First drive", firstDriveCommand);
        if (pinpoint != null) telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        if (follower != null && finitePose(follower.pose())) {
            Pose p = follower.pose();
            telemetry.addData("Pedro X/Y/H", "%.2f / %.2f in / %.1f deg",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("Field-center X/Y", "%.2f / %.2f in", p.x() - 72.0, p.y() - 72.0);
            double clearance = leftWallClearanceInches();
            telemetry.addData("Left-wall clearance", "%.2f in", clearance);
            if (clearance < LEFT_CLEARANCE_WARNING_INCHES) {
                telemetry.addLine("Close to left wall; follow clearance telemetry");
            }
            if (legStartPose != null) {
                telemetry.addData("Path displacement X/Y", "%.2f / %.2f in",
                        p.x() - legStartPose.x(), p.y() - legStartPose.y());
            }
            if (stage == Stage.PATH1) {
                telemetry.addData("Path 1 lateral error", "%.2f in", path1LateralError());
            }
            if (targetPose != null) {
                telemetry.addData("Target Pedro X/Y/H", "%.2f / %.2f / %.1f deg",
                        targetPose.x(), targetPose.y(), Math.toDegrees(targetPose.heading()));
            }
            if (stage == Stage.PATH1 || stage == Stage.PATH2 || stage == Stage.PATH3) {
                Pose reference = follower.closestPose();
                telemetry.addData("Path reference X/Y/H", "%.2f / %.2f / %.1f deg",
                        reference.x(), reference.y(), Math.toDegrees(reference.heading()));
            }
            if (stage != Stage.INIT && stage != Stage.DONE && stage != Stage.DWELL) {
                Map<String, Object> drive = follower.drivetrain.debug();
                telemetry.addData("Drive F/left/CCW", "%.3f / %.3f / %.3f",
                        drive.get("forward"), drive.get("strafe"), drive.get("turn"));
                telemetry.addData("Wheels FL/FR/BL/BR", "%.2f / %.2f / %.2f / %.2f",
                        drive.get("leftFrontWheelPower"), drive.get("rightFrontWheelPower"),
                        drive.get("leftBackWheelPower"), drive.get("rightBackWheelPower"));
                RobotLog.ii("RedHalfTwoTipTest", "%s | pose %.2f %.2f %.1f | velocity %s | drive %s",
                        stage, p.x(), p.y(), Math.toDegrees(p.heading()), follower.velocity(), drive);
            }
        }
        if (shootCommand != null) {
            telemetry.addData("Shot result / feed", "%s / %d ms",
                    shootCommand.getResult(), shootCommand.getFeedElapsedMs());
        }
        if (stage == Stage.DWELL) {
            telemetry.addData("Wait remaining", "%.1f s",
                    Math.max(0, DWELL_MS - elapsedMs(stageStartedNanos)) / 1000.0);
        }
        if (shooter != null && intake != null && transit != null) {
            telemetry.addData("Flywheel / transit / intake", "%.2f / %.2f / %.2f",
                    shooter.getRightPower(), transit.getRequestedPower(), intake.getRequestedPower());
        }
        telemetry.update();
    }
}
