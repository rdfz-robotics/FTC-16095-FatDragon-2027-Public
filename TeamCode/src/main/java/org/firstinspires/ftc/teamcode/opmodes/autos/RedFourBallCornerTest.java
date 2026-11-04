package org.firstinspires.ftc.teamcode.opmodes.autos;

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
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

/** Red-side test: one path, feed the four preloads at the corner, then one path. */
@Config
@Autonomous(name = "Red 4 Ball Corner Test v7", group = "Test")
public final class RedFourBallCornerTest extends LinearOpMode {
    private enum Stage { FIRST_PATH, SETTLE_AT_CORNER, PREPARE_SHOT, SHOOT, SECOND_PATH, DONE }

    private static final long FIRST_PATH_TIMEOUT_MS = 8_000;
    private static final long CORNER_SETTLE_TIMEOUT_MS = 1_500;
    private static final long SHOT_SETTLE_TIMEOUT_MS = 5_000;
    private static final long SHOOT_TIMEOUT_MS = 8_000;
    private static final long SECOND_PATH_TIMEOUT_MS = 7_000;
    private static final long AUTO_STOP_MS = 30_000;
    private static final long AIM_STABLE_MS = 200;
    private static final long PINPOINT_READY_WAIT_MS = 5_000;
    private static final double CORNER_TARGET_TOLERANCE_INCHES = 2.5;
    private static final double MAX_CORNER_ERROR_INCHES = 10.0;
    private static final String BUILD_TAG = "corner-test-v7-rear-shot-reversed-flywheel";
    private static final double FRONT_HEADING_DEGREES = 270.0;
    // Intake defines the front; the shooter and Limelight face the rear.
    private static final double SHOOTER_YAW_FROM_FRONT_DEGREES = 180.0;
    private static final double ROBOT_WIDTH_MM = 423.0;
    private static final double ROBOT_LENGTH_MM = 368.0;
    private static final double GRID_LINE_X_PEDRO_INCHES = 48.0;
    private static final double GRID_LINE_Y_PEDRO_INCHES = 120.0;
    private static final double START_Y_PEDRO_INCHES = 129.1295;
    private static final double START_X_PEDRO_INCHES =
            GRID_LINE_X_PEDRO_INCHES + ROBOT_WIDTH_MM / (2.0 * 25.4);
    private static final double FRONT_EDGE_ABOVE_GRID_Y_INCHES =
            START_Y_PEDRO_INCHES - ROBOT_LENGTH_MM / (2.0 * 25.4)
                    - GRID_LINE_Y_PEDRO_INCHES;
    private static final double FIRST_LEG_X_CHANGE_INCHES = 54.4376 - 54.1171;

    // Only the route uses field-center coordinates, in inches. The helper
    // converts them at the boundary expected by Pedro and the existing vision code.
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // With the chassis facing 270 degrees, its image-left side is on x=48.
    private final Pose start = fieldPose(
            START_X_PEDRO_INCHES - 72.0, START_Y_PEDRO_INCHES - 72.0, FRONT_HEADING_DEGREES);
    private final Pose firstCorner = fieldPose(
            START_X_PEDRO_INCHES + FIRST_LEG_X_CHANGE_INCHES - 72.0, 38.9725, FRONT_HEADING_DEGREES);
    private final Pose secondPoint = fieldPose(-45.97, 38.6426, FRONT_HEADING_DEGREES);

    // The robot has no ball-exit counter. Tune this from a filmed four-preload test.
    public static long feedDurationMs = 2_000;

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Transit transit;
    private GoBildaPinpointDriver pinpoint;
    private Stage stage = Stage.FIRST_PATH;
    private Pose shootHoldPose;
    private long autoStartedNanos;
    private long stageStartedNanos;
    private long aimStableSinceNanos = -1;
    private long feedStartedNanos = -1;
    private String status = "INIT";

    @Override
    public void runOpMode() throws InterruptedException {
        // FTCLib reset() clears the singleton. Obtain the instance that newly
        // constructed SubsystemBase objects will register with AFTER resetting.
        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();
        try {
            if (Constants.foresightConfig == null) {
                status = "Foresight config missing; deploy updated Constants.java";
                while (opModeInInit() && !isStopRequested()) {
                    showTelemetry();
                    idle();
                }
                waitForStart();
                while (opModeIsActive()) {
                    showTelemetry();
                    idle();
                }
                return;
            }

            follower = Constants.create(hardwareMap);
            // Reverse both flywheel motors for this test's observed hardware direction.
            shooter = new Shooter(hardwareMap,
                    ShooterConstants.leftShooterDirection.inverted(),
                    ShooterConstants.rightShooterDirection.inverted());
            intake = new Intake(hardwareMap);
            transit = new Transit(hardwareMap);
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                    Constants.localizerConfig.name.get());
            follower.setPose(start);

            while (opModeInInit() && !isStopRequested()) {
                follower.update();
                scheduler.run();
                showTelemetry();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            long pinpointWaitStartedNanos = System.nanoTime();
            while (opModeIsActive() && !pinpointReady()
                    && elapsedMs(pinpointWaitStartedNanos) < PINPOINT_READY_WAIT_MS) {
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
                autoStartedNanos = System.nanoTime();
                beginFirstPath();
            }
            while (opModeIsActive()) {
                if (stage != Stage.DONE) {
                    follower.update();
                    scheduler.run();
                    if (!pinpointReady()) {
                        finish("Pinpoint lost READY: " + pinpoint.getDeviceStatus());
                    } else if (elapsedMs(autoStartedNanos) >= AUTO_STOP_MS
                            || !finitePose(follower.pose())) {
                        finish("Stopped: invalid pose or AUTO timeout");
                    } else {
                        advance();
                    }
                }
                showTelemetry();
                idle();
            }
        } catch (RuntimeException error) {
            RobotLog.ee("RedFourBallCornerTest", error, "Autonomous failed");
            stopMechanisms();
            stopFollower();
            while (!isStopRequested() && (opModeInInit() || opModeIsActive())) {
                telemetry.addData("Build", BUILD_TAG);
                telemetry.addData("Auto error", error.toString());
                telemetry.update();
                idle();
            }
        } finally {
            scheduler.cancelAll();
            stopMechanisms();
            stopFollower();
            scheduler.reset();
        }
    }

    private void advance() {
        switch (stage) {
            case FIRST_PATH:
                if (!follower.isBusy()
                        || elapsedMs(stageStartedNanos) >= FIRST_PATH_TIMEOUT_MS) {
                    double error = cornerErrorInches();
                    if (error > MAX_CORNER_ERROR_INCHES) {
                        finish("Corner error " + roundedTenth(error) + " in; stopped");
                    } else if (error > CORNER_TARGET_TOLERANCE_INCHES) {
                        // Pedro can end a path after its endpoint timeout while
                        // still a few inches away. Give it one short correction.
                        follower.hold(firstCorner);
                        enter(Stage.SETTLE_AT_CORNER, "Correcting corner position");
                    } else {
                        prepareShot();
                    }
                }
                break;
            case SETTLE_AT_CORNER:
                double error = cornerErrorInches();
                if (error <= CORNER_TARGET_TOLERANCE_INCHES
                        || elapsedMs(stageStartedNanos) >= CORNER_SETTLE_TIMEOUT_MS) {
                    if (error > MAX_CORNER_ERROR_INCHES) {
                        finish("Corner error " + roundedTenth(error) + " in; stopped");
                    } else {
                        // Shoot from the measured pose even if the correction
                        // stops slightly short of the visualizer waypoint.
                        prepareShot();
                    }
                }
                break;
            case PREPARE_SHOT:
                if (elapsedMs(stageStartedNanos) >= SHOT_SETTLE_TIMEOUT_MS) {
                    finish("Aim timeout; heading error "
                            + roundedTenth(aimHeadingErrorDegrees()) + " deg");
                } else if (aimStable()) {
                    enter(Stage.SHOOT, "Spin up and feed preloads");
                }
                break;
            case SHOOT:
                shootAtCorner();
                break;
            case SECOND_PATH:
                if (elapsedMs(stageStartedNanos) >= SECOND_PATH_TIMEOUT_MS) {
                    finish("Second path timed out");
                } else if (!follower.isBusy()) {
                    finish("Two-path test complete");
                }
                break;
            case DONE:
                break;
        }
    }

    private void shootAtCorner() {
        if (elapsedMs(stageStartedNanos) >= SHOOT_TIMEOUT_MS) {
            finish("Shot preparation or feed timed out");
            return;
        }
        if (!aimWithinTolerance()) {
            finish("Aim drifted during shot");
            return;
        }

        double distanceCm = shotDistanceCm();
        if (!shooter.setDistanceTarget(distanceCm)) {
            finish("Shot calibration unavailable");
            return;
        }

        // Reuse the same subsystem calls as the manual shooting command.
        transit.requestFeed(shooter.isReady() && shooter.isPitchReady());
        boolean feeding = transit.getFiringState() == Transit.FiringState.FEEDING;
        intake.updateInputs(false, false, feeding);
        if (!feeding) {
            status = "Waiting for shooter/pitch or transit reverse";
            return;
        }
        if (feedStartedNanos < 0) feedStartedNanos = System.nanoTime();
        status = "Feeding four preloads: " + elapsedMs(feedStartedNanos) + " ms";
        if (elapsedMs(feedStartedNanos) >= Math.max(0, feedDurationMs)) {
            // Match the manual shooting command when finishing an open-gate feed.
            intake.clearBallCount();
            stopMechanisms();
            follower.stop();
            Pose actual = follower.pose();
            // Keep the shot heading while moving toward decreasing field X.
            double shotHeading = Math.toDegrees(shootHoldPose.heading());
            Pose strafeStart = poseFactory.of(actual.x(), actual.y(), shotHeading);
            Pose strafeEnd = poseFactory.of(secondPoint.x(), secondPoint.y(), shotHeading);
            Path path2 = line(strafeStart, strafeEnd).linear(strafeStart, strafeEnd);
            follower.follow(path2);
            enter(Stage.SECOND_PATH, "Strafing left along second path");
        }
    }

    private boolean aimStable() {
        if (!aimWithinTolerance()) {
            aimStableSinceNanos = -1;
            return false;
        }
        if (aimStableSinceNanos < 0) aimStableSinceNanos = System.nanoTime();
        return elapsedMs(aimStableSinceNanos) >= AIM_STABLE_MS;
    }

    private boolean aimWithinTolerance() {
        Pose actual = follower.pose();
        return Math.hypot(actual.x() - shootHoldPose.x(),
                actual.y() - shootHoldPose.y()) <= 1.5
                && aimHeadingErrorDegrees() <= 3.0;
    }

    private void prepareShot() {
        Pose actual = follower.pose();
        // Turn the rear-mounted shooter toward the red target from the measured pose.
        shootHoldPose = poseFactory.of(actual.x(), actual.y(), shotHeadingDegrees(actual));
        if (!shooter.setDistanceTarget(shotDistanceCm())) {
            finish("Shot calibration unavailable");
            return;
        }
        aimStableSinceNanos = -1;
        follower.hold(shootHoldPose);
        enter(Stage.PREPARE_SHOT, "Aiming rear shooter; flywheel spinning up");
    }

    private double cornerErrorInches() {
        Pose actual = follower.pose();
        return Math.hypot(actual.x() - firstCorner.x(), actual.y() - firstCorner.y());
    }

    private double aimHeadingErrorDegrees() {
        return Math.abs(Math.toDegrees(Math.IEEEremainder(
                follower.pose().heading() - shootHoldPose.heading(), 2.0 * Math.PI)));
    }

    private static double roundedTenth(double value) {
        return Math.round(value * 10.0) / 10.0;
    }

    private void beginFirstPath() {
        follower.follow(line(start, firstCorner).linear(start, firstCorner));
        enter(Stage.FIRST_PATH, "Following first path");
    }

    private void enter(Stage next, String message) {
        stage = next;
        stageStartedNanos = System.nanoTime();
        status = message;
    }

    /** Pedro takes a lower-left origin; this test defines its waypoints from center. */
    private Pose fieldPose(double x, double y, double headingDegrees) {
        return poseFactory.of(x + 72.0, y + 72.0, headingDegrees);
    }

    private double shotHeadingDegrees(Pose shotPose) {
        double[] target = VisionConstants.cellTargetFieldXYInches[
                CellCluster.RED_OPPOSITE_AUDIENCE.getTuningIndex()];
        double targetBearing = Math.toDegrees(Math.atan2(
                target[1] - (shotPose.y() - 72.0),
                target[0] - (shotPose.x() - 72.0)));
        double chassisHeading = targetBearing - SHOOTER_YAW_FROM_FRONT_DEGREES;
        return ((chassisHeading % 360.0) + 360.0) % 360.0;
    }

    private double shotDistanceCm() {
        Pose shotPose = shootHoldPose == null ? firstCorner : shootHoldPose;
        double[] tag = VisionConstants.tagClusterCenterFieldXYInches[
                CellCluster.RED_OPPOSITE_AUDIENCE.getTuningIndex()];
        return DistanceUnit.INCH.toCm(Math.hypot(
                tag[0] - (shotPose.x() - 72.0),
                tag[1] - (shotPose.y() - 72.0)));
    }

    private boolean pinpointReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    private static boolean finitePose(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private void finish(String message) {
        status = message;
        stopMechanisms();
        stopFollower();
        stage = Stage.DONE;
    }

    private void stopMechanisms() {
        if (transit != null) transit.cancelFeeding();
        if (intake != null) intake.stop();
        if (shooter != null) shooter.stop();
    }

    private void stopFollower() {
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop();
        }
    }

    private static long elapsedMs(long startedNanos) {
        return (System.nanoTime() - startedNanos) / 1_000_000L;
    }

    private void showTelemetry() {
        telemetry.addData("Build", BUILD_TAG);
        telemetry.addData("Foresight", Constants.foresightConfig == null ? "MISSING" : "LOADED");
        if (pinpoint != null) telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Place", "Image-left chassis side on Pedro x=48 grid, face 270 deg");
        telemetry.addData("Robot width/length", "%.0f / %.0f mm", ROBOT_WIDTH_MM, ROBOT_LENGTH_MM);
        telemetry.addData("Start Pedro X/Y", "%.2f / %.2f in", start.x(), start.y());
        telemetry.addData("Front edge above y=120", "%.2f in if pose is chassis center",
                FRONT_EDGE_ABOVE_GRID_Y_INCHES);
        telemetry.addData("Stage", stage);
        telemetry.addData("Status", status);
        if (follower != null && finitePose(follower.pose())) {
            Pose pose = follower.pose();
            telemetry.addData("Follower", follower.mode());
            telemetry.addData("Field-center X/Y/H", "%.2f / %.2f in / %.1f deg",
                    pose.x() - 72.0, pose.y() - 72.0,
                    Math.toDegrees(pose.heading()));
            telemetry.addData("Corner error", "%.2f in", cornerErrorInches());
            if (shootHoldPose != null) {
                telemetry.addData("Aim heading error", "%.1f deg", aimHeadingErrorDegrees());
            }
        }
        if (shooter != null && transit != null) {
            telemetry.addData("Shooter / transit", "%s / %s",
                    shooter.isReady() && shooter.isPitchReady(), transit.getFiringState());
            telemetry.addData("Shooter velocity", "%.0f / %.0f ticks/s",
                    shooter.getRightVelocity(), shooter.getTargetVelocity());
            telemetry.addData("Flywheel power L/R", "%.2f / %.2f",
                    shooter.getLeftPower(), shooter.getRightPower());
            telemetry.addData("Transit power / gate", "%.2f / %s",
                    transit.getRequestedPower(), transit.getState());
            telemetry.addData("Intake power", "%.2f", intake.getRequestedPower());
            telemetry.addData("Shot distance", "%.1f cm", shotDistanceCm());
            telemetry.addData("Feed duration", "%d / %d ms",
                    feedStartedNanos < 0 ? 0 : elapsedMs(feedStartedNanos), feedDurationMs);
        }
        telemetry.update();
    }
}
