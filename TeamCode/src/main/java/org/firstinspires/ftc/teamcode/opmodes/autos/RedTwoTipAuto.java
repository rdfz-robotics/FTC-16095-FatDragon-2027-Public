package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveState;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/**
 * Red-side, four-preload two-tip attempt. No collection is allowed: the current
 * intake sensor cannot prove how many of the four preloads remain in the robot.
 *
 * <p>All route coordinates below use the field center as (0, 0), in inches:
 * +X to the audience and +Y toward the blue HIVE. Only fieldPose() converts
 * them to Pedro's lower-left-corner coordinates. Check the actual robot
 * footprint, wall contact, FLOWER clearance, and shot calibration before use.</p>
 */
@Autonomous(name = "Red Two Tip - 4 Preload", group = "Auto")
public final class RedTwoTipAuto extends LinearOpMode {
    private enum Stage {
        TO_FIRST_SHOT, SELECT_FIRST_CELL, TO_FIRST_CELL, FIRE_FIRST,
        SELECT_SECOND_CELL, TO_SECOND_CELL, FIRE_SECOND,
        TO_PARK_LANE, TO_PARK, DONE
    }

    // Start at the red rear wall, outside the red GARDEN and LOADING ZONE.
    // Field Y=-63 assumes an approximately 18-inch-long robot touching the wall.
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = fieldPose(-18, -63, 270);
    private final Pose oppositeShot = fieldPose(-18, -38, 270);
    private final Pose audienceShot = fieldPose(-20, -37, 213);
    private final Pose parkLane = fieldPose(-43, -2, 270);
    private final Pose park = fieldPose(-57, 31, 270);

    private static final long PARK_START_MS = 21_000;
    private static final long HARD_STOP_MS = 29_000;
    private static final long FLIP_CONFIRM_MS = 250;
    private static final long FIRST_FIRE_LIMIT_MS = 6_000;
    private static final long SECOND_FIRE_LIMIT_MS = 4_500;

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Transit transit;
    private Vision vision;
    private AutoHiveTarget oppositeCell;
    private AutoHiveTarget audienceCell;
    private AutoHiveTarget selectedCell;
    private GoBildaPinpointDriver pinpoint;
    private Stage stage = Stage.TO_FIRST_SHOT;
    private Stage afterPath;
    private long autoStartedNanos;
    private long stageStartedNanos;
    private long pathTimeoutMs;
    private long flipObservedNanos = -1;
    private int observedTips;
    private String status = "INIT";

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.reset();
        try {
            if (Constants.foresightConfig == null) {
                while (opModeInInit() && !isStopRequested()) {
                    telemetry.addLine("Run AutoTune > Foresight Tuner first.");
                    telemetry.addLine("Paste its Java export into pedro/Constants.java and rebuild.");
                    telemetry.update();
                    idle();
                }
                return;
            }

            follower = Constants.create(hardwareMap);
            shooter = new Shooter(hardwareMap);
            intake = new Intake(hardwareMap);
            transit = new Transit(hardwareMap);
            vision = new Vision(hardwareMap, Alliance.RED);
            oppositeCell = new AutoHiveTarget(vision, CellCluster.RED_OPPOSITE_AUDIENCE);
            audienceCell = new AutoHiveTarget(vision, CellCluster.RED_AUDIENCE);
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                    Constants.localizerConfig.name.get());

            follower.setPose(start);
            while (opModeInInit() && !isStopRequested()) {
                updateSensors(scheduler);
                showTelemetry();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            if (!pinpointReady()) {
                status = "Pinpoint is not READY";
                showTelemetry();
                return;
            }

            follower.setPose(start);
            autoStartedNanos = System.nanoTime();
            follow(line(start, oppositeShot).linear(start, oppositeShot),
                    Stage.TO_FIRST_SHOT, Stage.SELECT_FIRST_CELL, 5_000);
            while (opModeIsActive() && stage != Stage.DONE) {
                updateSensors(scheduler);
                long elapsedMs = elapsedMs(autoStartedNanos);
                if (!pinpointReady() || !insideRedSafetyEnvelope(follower.pose())
                        || elapsedMs >= HARD_STOP_MS) {
                    status = "Stopped: pose, Pinpoint, or AUTO time limit";
                    finish();
                } else if (elapsedMs >= PARK_START_MS && !isParking()) {
                    beginPark();
                } else {
                    advance();
                }
                showTelemetry();
                idle();
            }
        } finally {
            scheduler.cancelAll();
            stopMechanisms();
            if (vision != null) vision.stop();
            stopFollower();
            scheduler.reset();
        }
    }

    private void updateSensors(CommandScheduler scheduler) {
        follower.update();
        scheduler.run();
        oppositeCell.update(follower.pose());
        audienceCell.update(follower.pose());
    }

    private void advance() {
        switch (stage) {
            case TO_FIRST_SHOT:
            case TO_FIRST_CELL:
            case TO_SECOND_CELL:
            case TO_PARK_LANE:
            case TO_PARK:
                advancePath();
                break;
            case SELECT_FIRST_CELL:
                selectCell(false);
                break;
            case FIRE_FIRST:
                fire(false);
                break;
            case SELECT_SECOND_CELL:
                selectCell(true);
                break;
            case FIRE_SECOND:
                fire(true);
                break;
            case DONE:
                break;
        }
    }

    private void selectCell(boolean second) {
        stopMechanisms();
        AutoHiveTarget high = currentlyHighCell();
        if (high != null && (!second || high != selectedCell)) {
            selectedCell = high;
            Pose shot = high == oppositeCell ? oppositeShot : audienceShot;
            Stage fireStage = second ? Stage.FIRE_SECOND : Stage.FIRE_FIRST;
            Stage travelStage = second ? Stage.TO_SECOND_CELL : Stage.TO_FIRST_CELL;
            if (atPose(shot)) {
                enter(fireStage, "Shooting " + (second ? "second" : "first") + " upper CELL");
            } else {
                Pose current = follower.pose();
                follow(line(current, shot).linear(current, shot),
                        travelStage, fireStage, 3_000);
            }
        } else if (elapsedMs(stageStartedNanos) >= 1_500) {
            status = "No confirmed upper red CELL";
            beginPark();
        }
    }

    private void fire(boolean second) {
        AutoHiveTarget other = selectedCell == oppositeCell ? audienceCell : oppositeCell;
        if (selectedCell.getState() == HiveState.LOW && other.getState() == HiveState.HIGH) {
            if (flipObservedNanos < 0) flipObservedNanos = System.nanoTime();
            if (elapsedMs(flipObservedNanos) >= FLIP_CONFIRM_MS) {
                observedTips++;
                stopMechanisms();
                if (second) beginPark();
                else enter(Stage.SELECT_SECOND_CELL, "First tip observed; select new upper CELL");
                return;
            }
        } else {
            flipObservedNanos = -1;
        }

        long limit = second ? SECOND_FIRE_LIMIT_MS : FIRST_FIRE_LIMIT_MS;
        if (elapsedMs(stageStartedNanos) >= limit) {
            status = "Shot timed out; parking";
            beginPark();
            return;
        }
        if (selectedCell.getState() != HiveState.HIGH || !atPose(shotPose(selectedCell))
                || !Double.isFinite(selectedCell.getDistanceCm())) {
            stopMechanisms();
            status = "Waiting for upper CELL / pose / distance";
            return;
        }

        // The same Shooter and Transit controls are used by the manual commands.
        // Do not continue feeding while the HIVE is moving or its state is unknown.
        if (!shooter.setDistanceTarget(selectedCell.getDistanceCm())) {
            transit.cancelFeeding();
            status = "Waiting for a calibrated shot target";
            return;
        }
        transit.requestFeed(shooter.isReady() && shooter.isPitchReady());
        status = "Attempting tip " + (second ? "2" : "1") + ": "
                + transit.getFiringState();
    }

    private void advancePath() {
        if (elapsedMs(stageStartedNanos) >= pathTimeoutMs) {
            status = "Path timeout: " + stage;
            if (isParking()) finish();
            else beginPark();
            return;
        }
        if (follower.isBusy()) return;
        stopFollower();
        if (stage == Stage.TO_PARK_LANE) {
            Pose current = follower.pose();
            follow(line(current, park).linear(current, park),
                    Stage.TO_PARK, Stage.DONE, 4_000);
        } else if (stage == Stage.TO_PARK) {
            finish();
        } else {
            enter(afterPath, "Arrived: " + afterPath);
        }
    }

    private void beginPark() {
        if (isParking() || stage == Stage.DONE) return;
        stopMechanisms();
        stopFollower();
        Pose current = follower.pose();
        Pose control = fieldPose(-36, -24, 270);
        follow(curve(current, control, parkLane).linear(current, parkLane),
                Stage.TO_PARK_LANE, Stage.TO_PARK, 4_000);
        status = "Parking in red LOADING ZONE";
    }

    private void follow(Path path, Stage travel, Stage next, long timeoutMs) {
        follower.follow(path);
        stage = travel;
        afterPath = next;
        pathTimeoutMs = timeoutMs;
        stageStartedNanos = System.nanoTime();
        flipObservedNanos = -1;
        status = "Following " + travel;
    }

    private void enter(Stage next, String description) {
        stage = next;
        stageStartedNanos = System.nanoTime();
        flipObservedNanos = -1;
        status = description;
    }

    private AutoHiveTarget currentlyHighCell() {
        boolean oppositeHigh = oppositeCell.getState() == HiveState.HIGH;
        boolean audienceHigh = audienceCell.getState() == HiveState.HIGH;
        if (oppositeHigh == audienceHigh) return null;
        return oppositeHigh ? oppositeCell : audienceCell;
    }

    private Pose shotPose(AutoHiveTarget cell) {
        return cell == oppositeCell ? oppositeShot : audienceShot;
    }

    /** Convert the field-center coordinates used by this route at the Pedro boundary. */
    private Pose fieldPose(double fieldX, double fieldY, double headingDegrees) {
        return poseFactory.of(fieldX + 72.0, fieldY + 72.0, headingDegrees);
    }

    private boolean atPose(Pose desired) {
        Pose current = follower.pose();
        return Double.isFinite(current.x()) && Double.isFinite(current.y())
                && Double.isFinite(current.heading())
                && Math.hypot(current.x() - desired.x(), current.y() - desired.y()) <= 2.5
                && Math.abs(Math.toDegrees(Math.IEEEremainder(
                        current.heading() - desired.heading(), 2.0 * Math.PI))) <= 4.0;
    }

    // Reserve clearance for the robot footprint and pose error at the centerline.
    // This is a software stop, not a substitute for measuring the actual robot.
    private static boolean insideRedSafetyEnvelope(Pose pose) {
        if (pose == null || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading())) return false;
        double fieldX = pose.x() - 72.0;
        double fieldY = pose.y() - 72.0;
        return fieldX >= -63.0 && fieldX <= -12.0
                && fieldY >= -64.0 && fieldY <= 63.0;
    }

    private boolean pinpointReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    private boolean isParking() {
        return stage == Stage.TO_PARK_LANE || stage == Stage.TO_PARK;
    }

    private void finish() {
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
        if (follower == null) return;
        follower.stop();
        follower.drivetrain.stop();
    }

    private static long elapsedMs(long sinceNanos) {
        return (System.nanoTime() - sinceNanos) / 1_000_000L;
    }

    private void showTelemetry() {
        telemetry.addData("Red auto", status);
        telemetry.addData("Stage / observed tips", "%s / %d", stage, observedTips);
        if (follower != null) {
            Pose pose = follower.pose();
            telemetry.addData("Field-center x/y/h (in)", "%.1f / %.1f / %.1f",
                    pose.x() - 72.0, pose.y() - 72.0,
                    Math.toDegrees(pose.heading()));
        }
        if (oppositeCell != null && audienceCell != null) {
            telemetry.addData("Red CELL opposite/audience", "%s / %s",
                    oppositeCell.getState(), audienceCell.getState());
            telemetry.addData("HIVE observation", oppositeCell.getStatus());
        }
        if (shooter != null && transit != null) {
            telemetry.addData("Shooter ready / feed", "%s / %s",
                    shooter.isReady() && shooter.isPitchReady(), transit.getFiringState());
        }
        telemetry.addLine("Four preloads only: GARDEN pickup disabled until empty-magazine timing is measured.");
        telemetry.update();
    }
}
