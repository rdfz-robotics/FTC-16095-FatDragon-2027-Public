package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.commands.ShootCommand.FeedPermission;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.PositionHoldController;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveState;

import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Pedro aiming adapter around the same mechanism command used by TeleOp.
 * Run after a path; keep calling follower.update() and CommandScheduler.run().
 * Vision must be registered with that scheduler. The route exclusively lends
 * this command its follower until end(): do not call follow/hold alongside it.
 * FTCLib protects the three mechanisms, but Follower is not an FTCLib subsystem.
 */
public class AutoShootCommand extends SequentialCommandGroup {
    // Keep the Dong route's shot-drift limits and 200 ms settling requirement.
    private static final long SETTLE_MS = 200;
    private static final double MAX_POSITION_ERROR_INCHES = 1.5;
    private static final double MAX_FEED_YAW_ERROR_DEGREES = 3.0;

    private final Follower follower;
    private final AutoHiveTarget target;
    private final BooleanSupplier localizationReady;
    private final boolean waitForHigh;
    private final ShootCommand shooting;
    private final AutoAimController autoAim = new AutoAimController();
    private final PositionHoldController positionHold = new PositionHoldController();

    private FeedPermission permission = FeedPermission.INVALID_TARGET;
    private boolean aimReady;
    private boolean active;
    private Pose anchor;
    private long stationarySinceNanos = -1;
    private double distanceCm = Double.NaN;
    private double yawErrorDegrees = Double.NaN;
    private double positionErrorInches = Double.NaN;
    private DrivePowers powers;
    private String status = "Not started";

    public AutoShootCommand(Follower follower, Shooter shooter, Intake intake, Transit transit,
                            AutoHiveTarget target, BooleanSupplier localizationReady,
                            long feedDurationMs, long timeoutMs, boolean waitForHigh) {
        this.follower = Objects.requireNonNull(follower);
        this.target = Objects.requireNonNull(target);
        this.localizationReady = Objects.requireNonNull(localizationReady);
        this.waitForHigh = waitForHigh;
        shooting = new ShootCommand(shooter, intake, transit, () -> distanceCm, () -> aimReady,
                () -> permission, true, feedDurationMs, timeoutMs);
        // A real composition owns the child's requirements and lifecycle. Never
        // separately schedule shooting or call its lifecycle from another owner.
        addCommands(shooting);
    }

    @Override
    public void initialize() {
        permission = FeedPermission.INVALID_TARGET;
        aimReady = false;
        anchor = null;
        stationarySinceNanos = -1;
        distanceCm = yawErrorDegrees = positionErrorInches = Double.NaN;
        powers = null;
        status = "Waiting for stationary chassis";
        autoAim.reset();
        positionHold.reset();
        stopFollower();
        active = true;
        super.initialize(); // Preserves preheat, closes the gate and stops intake.
    }

    @Override
    public void execute() {
        if (!active || shooting.isFinished()) return;
        updateAim(); // Compute this cycle's permission and lock before the child runs.
        super.execute();
        if (shooting.isFinished()) {
            stopFollower();
        } else if (!shooting.hasValidTarget()) {
            if (permission != FeedPermission.INVALID_TARGET) {
                waitForValidTarget("Waiting for a valid shot target / calibration");
            }
        } else if (powers != null) {
            follower.manual(powers);
            follower.drivetrain.drive(powers, true);
            if (shooting.canFire()) status = "Shooting: " + shooting.getFeedElapsedMs() + " ms";
            else if (permission == FeedPermission.ALLOW && aimReady) {
                status = "Waiting for shooter / pitch / feed preparation";
            }
        }
    }

    private void updateAim() {
        aimReady = false;
        powers = null;
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        if (!localizationReady.getAsBoolean() || pose == null || velocity == null
                || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading()) || !Double.isFinite(velocity.linSpeed())
                || !Double.isFinite(velocity.omega)) {
            waitForValidTarget("Waiting for valid localization");
            return;
        }
        target.update(pose);
        distanceCm = target.getDistanceCm();
        yawErrorDegrees = target.getYawErrorDegrees();
        double maxSpeed = DriveConstants.positionLockMaxSpeedInchesPerSecond;
        if (!Double.isFinite(distanceCm) || !Double.isFinite(yawErrorDegrees)
                || !Double.isFinite(maxSpeed) || maxSpeed < 0) {
            waitForValidTarget("Waiting for a valid shot target / configuration");
            return;
        }

        if (anchor == null) {
            stopFollower();
            permission = FeedPermission.WAIT_FOR_START;
            status = "Waiting for stationary chassis";
            if (velocity.linSpeed() > maxSpeed) {
                stationarySinceNanos = -1;
                return;
            }
            if (stationarySinceNanos < 0) stationarySinceNanos = System.nanoTime();
            if ((System.nanoTime() - stationarySinceNanos) / 1_000_000L < SETTLE_MS) return;
            anchor = new Pose(pose.x(), pose.y(), pose.heading());
        }

        positionErrorInches = Math.hypot(pose.x() - anchor.x(), pose.y() - anchor.y());
        boolean committed = shooting.getFiringState() == Transit.FiringState.REVERSING
                || shooting.getFiringState() == Transit.FiringState.FEEDING;
        if (positionErrorInches > MAX_POSITION_ERROR_INCHES
                || (committed && (Math.abs(yawErrorDegrees) > MAX_FEED_YAW_ERROR_DEGREES
                || velocity.linSpeed() > maxSpeed))) {
            permission = FeedPermission.ABORT;
            status = "Shot aborted: chassis / aim drift";
            stopFollower();
            return;
        }

        double turnPower = autoAim.calculateTurnPower(true, yawErrorDegrees);
        positionHold.update(AutoHiveTarget.toHiveFieldPose(pose), true, velocity.linSpeed());
        // Pedro is left/CCW-positive; the shared TeleOp controllers are right/CW-positive.
        powers = new DrivePowers(positionHold.getForwardPower(),
                -positionHold.getStrafePower(), -turnPower);
        aimReady = autoAim.isLocked() && velocity.linSpeed() <= maxSpeed;
        permission = waitForHigh && !shooting.canFire() && target.getState() != HiveState.HIGH
                ? FeedPermission.WAIT_FOR_START : FeedPermission.ALLOW;
        status = permission == FeedPermission.WAIT_FOR_START
                ? "Waiting for HIGH: " + target.getState() : "Aiming";
    }

    private void waitForValidTarget(String reason) {
        permission = FeedPermission.INVALID_TARGET;
        aimReady = false;
        distanceCm = Double.NaN;
        powers = null;
        stationarySinceNanos = -1;
        autoAim.reset();
        positionHold.reset();
        stopFollower();
        status = reason;
    }

    @Override
    public boolean isFinished() {
        // Keep completion observable even after FTCLib resets the group's index in end().
        return shooting.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (active) {
            super.end(interrupted && !shooting.isFinished());
            active = false;
        }
        autoAim.reset();
        positionHold.reset();
        stopFollower();
        switch (shooting.getResult()) {
            case SUCCEEDED: status = "Shot complete"; break;
            case TIMED_OUT: status = "Shot timed out"; break;
            case INTERRUPTED: status = "Shot interrupted"; break;
            default: break; // Retain the specific abort reason.
        }
    }

    private void stopFollower() {
        follower.stop();
        follower.drivetrain.stop();
    }

    public String getStatus() { return status; }
    public boolean hasTimedOut() { return shooting.getResult() == ShootCommand.Result.TIMED_OUT; }
    public ShootCommand.Result getResult() { return shooting.getResult(); }
    public Transit.FiringState getFiringState() { return shooting.getFiringState(); }
    public long getFeedElapsedMs() { return shooting.getFeedElapsedMs(); }
    public double getDistanceCm() { return distanceCm; }
    public double getYawErrorDegrees() { return yawErrorDegrees; }
    public double getPositionErrorInches() { return positionErrorInches; }
}
