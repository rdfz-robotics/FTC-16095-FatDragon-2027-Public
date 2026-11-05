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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

import java.util.Locale;

/** Test 2: shoot once, travel to the next waypoint, stop for 3 s, cross the field. */
@Config
@Autonomous(name = "Red Half Two Tip Test 2 v3", group = "Test")
public final class RedHalfTwoTipTest extends LinearOpMode {
    private enum Stage { INIT, PATH1, SHOOT, RESTORE_HEADING, PATH2, DWELL, PATH3, DONE }

    private static final String BUILD_TAG = "red-half-two-tip-test2-v3-wall-clearance";
    private static final long AUTO_TIMEOUT_MS = 30_000;
    private static final long PINPOINT_WAIT_MS = 5_000;
    private static final long SHOOT_TIMEOUT_MS = 13_000;
    private static final long RESTORE_TIMEOUT_MS = 5_000;
    private static final long DWELL_MS = 3_000;
    private static final long STABLE_MS = 200;
    private static final double ARRIVAL_TOLERANCE_INCHES = 2.5;
    public static long feedDurationMs = 2_000;
    private static final double PATH_SPEED_FRACTION = 0.35;
    private static final double ROBOT_HALF_WIDTH_INCHES = 423.0 / (2.0 * 25.4);
    private static final double ROBOT_HALF_LENGTH_INCHES = 368.0 / (2.0 * 25.4);
    private static final double MIN_LEFT_CLEARANCE_INCHES = 4.0;
    private static String previousRun = "No previous test in this Robot Controller session";

    private final PoseFactory poses = PoseFactory.degrees();
    // Field-center coordinates in inches. fieldPose adds 72 at the Pedro boundary.
    // Waypoint positions follow the Visualizer export. The first leg keeps 90 degrees
    // during travel, then turns in place to aim the rear shooter at the first waypoint.
    private final Pose start = fieldPose(-18.0058, -64.0775, 90);
    private final Pose firstTravelEnd = fieldPose(-18.1072, -40.4166, 90);
    private final Pose firstShot = fieldPose(-18.1072, -40.4166, 270);
    private final Pose dwellPoint = fieldPose(-56.8517, -60.5817, 270);
    private final Pose oppositePoint = fieldPose(-14.6715, 56.6085, 90);
    // Keep the curve's control X at the dwell point so it starts toward the field interior.
    private final Pose crossingControl = fieldPose(-56.8517, 5.1069, 0);

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

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        scheduler = CommandScheduler.getInstance();
        try {
            if (Constants.foresightConfig == null) {
                throw new IllegalStateException("Foresight config missing; deploy tuned Constants.java");
            }
            follower = Constants.create(hardwareMap);
            shooter = new Shooter(hardwareMap);
            intake = new Intake(hardwareMap);
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
            } else {
                follower.setPose(start);
                follow(path1(), Stage.PATH1, "First path to shooting waypoint");
            }
            while (opModeIsActive()) {
                if (stage != Stage.DONE) {
                    follower.update();
                    scheduler.run();
                    if (!pinpointReady() || !finitePose(follower.pose())) {
                        finish("Stopped: invalid pose or Pinpoint " + pinpoint.getDeviceStatus());
                    } else if (elapsedMs(autoStartedNanos) >= AUTO_TIMEOUT_MS) {
                        finish("30-second test timeout");
                    } else if (checkTravelDirection()) {
                        advance();
                    }
                }
                showTelemetry();
                idle();
            }
        } catch (RuntimeException error) {
            RobotLog.ee("RedHalfTwoTipTest", error, "Autonomous failed");
            stopAll();
            while (!isStopRequested() && (opModeInInit() || opModeIsActive())) {
                telemetry.addData("Build", BUILD_TAG);
                telemetry.addData("Auto error", error.toString());
                telemetry.update();
                idle();
            }
        } finally {
            recordRunSummary();
            scheduler.cancelAll();
            stopAll();
            scheduler.reset();
        }
    }

    private void advance() {
        switch (stage) {
            case PATH1:
                if (pathArrived(firstTravelEnd, 8_000)) beginShot();
                break;
            case SHOOT:
                advanceShot();
                break;
            case RESTORE_HEADING:
                if (holdStable(firstShot, ARRIVAL_TOLERANCE_INCHES)) {
                    follow(path2(), Stage.PATH2, "Second path; mechanisms stopped");
                } else if (elapsedMs(stageStartedNanos) >= RESTORE_TIMEOUT_MS) {
                    finish("Could not restore path 2 start pose / 270 deg heading");
                }
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
        Pose actual = follower.pose();
        double[] target = VisionConstants.cellTargetFieldXYInches[
                CellCluster.RED_OPPOSITE_AUDIENCE.getTuningIndex()];
        double bearing = Math.toDegrees(Math.atan2(target[1] - (actual.y() - 72.0),
                target[0] - (actual.x() - 72.0)));
        // The shooter faces the rear, in the same direction as the Limelight.
        double chassisHeading = ((bearing - 180.0) % 360.0 + 360.0) % 360.0;
        holdPose = poses.of(actual.x(), actual.y(), chassisHeading);
        targetPose = holdPose;
        follower.hold(holdPose);
        enter(Stage.SHOOT, "Turning rear shooter to red target and firing preloads");
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
        // ShootCommand.end() has stopped all mechanisms. Restore the exported
        // path2 start pose before following its constant 270-degree heading.
        targetPose = firstShot;
        follower.hold(firstShot);
        enter(Stage.RESTORE_HEADING, "Restoring path 2 start heading to 270 deg");
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
        // Match the CELL-opening distance used by the shared Shooter calibration.
        double[] target = VisionConstants.cellTargetFieldXYInches[
                CellCluster.RED_OPPOSITE_AUDIENCE.getTuningIndex()];
        return DistanceUnit.INCH.toCm(Math.hypot(target[0] - (holdPose.x() - 72.0),
                target[1] - (holdPose.y() - 72.0)));
    }

    private void follow(Path path, Stage next, String message) {
        legStartPose = follower.pose();
        targetPose = path.endPose();
        follower.follow(path.with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED_FRACTION)));
        enter(next, message);
    }

    private boolean checkTravelDirection() {
        if (leftWallClearanceInches() < MIN_LEFT_CLEARANCE_INCHES) {
            finish("Left-wall clearance below 4 in; check actual position and odometry");
            return false;
        }
        if (legStartPose == null) return true;
        double dx = follower.pose().x() - legStartPose.x();
        double dy = follower.pose().y() - legStartPose.y();
        if (stage == Stage.PATH1 && (Math.abs(dx) > 3.0 || dy < -2.0)) {
            finish("PATH1 wrong movement: expect +Y, nearly fixed X; check placement/localizer");
            return false;
        }
        if (stage == Stage.PATH2 && (dx > 2.0 || dy > 2.0)) {
            finish("PATH2 wrong movement: expect -X and -Y; check localizer/drivetrain");
            return false;
        }
        return true;
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
        previousRun = stage + " | " + status + " | " + poseText;
        RobotLog.ii("RedHalfTwoTipTest", previousRun);
    }

    private void enter(Stage next, String message) {
        stage = next;
        status = message;
        stageStartedNanos = System.nanoTime();
        stableSinceNanos = -1;
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

    private Pose fieldPose(double x, double y, double headingDegrees) {
        return poses.of(x + 72.0, y + 72.0, headingDegrees);
    }

    public Path path1() { return line(start, firstTravelEnd).linear(start, firstTravelEnd); }
    public Path path2() { return line(firstShot, dwellPoint).linear(firstShot, dwellPoint); }
    public Path path3() { return curve(dwellPoint, crossingControl, oppositePoint).linear(dwellPoint, oppositePoint); }

    private void showTelemetry() {
        telemetry.addData("Build", BUILD_TAG);
        if (stage == Stage.INIT) telemetry.addData("Previous run", previousRun);
        telemetry.addData("Stage", stage);
        telemetry.addData("Status", status);
        if (pinpoint != null) telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        if (follower != null && finitePose(follower.pose())) {
            Pose p = follower.pose();
            telemetry.addData("Pedro X/Y/H", "%.2f / %.2f in / %.1f deg",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("Field-center X/Y", "%.2f / %.2f in", p.x() - 72.0, p.y() - 72.0);
            telemetry.addData("Left-wall clearance", "%.2f in", leftWallClearanceInches());
            if (legStartPose != null) {
                telemetry.addData("Path displacement X/Y", "%.2f / %.2f in",
                        p.x() - legStartPose.x(), p.y() - legStartPose.y());
            }
            if (targetPose != null) {
                telemetry.addData("Target Pedro X/Y/H", "%.2f / %.2f / %.1f deg",
                        targetPose.x(), targetPose.y(), Math.toDegrees(targetPose.heading()));
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
