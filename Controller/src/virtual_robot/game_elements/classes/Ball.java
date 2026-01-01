package virtual_robot.game_elements.classes;

import javafx.fxml.FXML;
import javafx.scene.shape.Circle;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Transform;
import virtual_robot.controller.Filters;
import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;
import virtual_robot.games.Decode;
import javafx.application.Platform;
import javafx.scene.paint.Color;
import java.util.ArrayList;
import java.util.List;
import org.dyn4j.dynamics.BodyFixture;


/**
 * Simple pushable ball game element.
 * - Has a circular physics body (dyn4j)
 * - Renders as a JavaFX Circle from ball.fxml
 * - Collides with robots/walls/other collidable objects
 */
@GameElementConfig(name = "Ball", filename = "ball", forGame = Decode.class, numInstances = 24)
public class Ball extends VirtualGameElement {

    // Decode will populate this list in its initialize() (pattern used by CenterStage).
    public static final List<Ball> balls = new ArrayList<>();

    // Unique collision category (not used elsewhere in this repo)
    public static final long BALL_CATEGORY = 16384L;

    // Collide with everything
    public static final CategoryFilter BALL_FILTER =
            new CategoryFilter(BALL_CATEGORY, Filters.MASK_ALL);

    // Size: radius in inches (change if you want bigger/smaller)
    public static final double BALL_RADIUS_INCHES = 2.0;
    // Respawn point (inches) if ball leaves field
    private static final double RESPAWN_X_IN = 0.0;
    private static final double RESPAWN_Y_IN = -60.0;
    // Field bounds (inches). Tune if your sim uses slightly different limits.
    private static final double X_MIN_IN = -72.0;
    private static final double X_MAX_IN =  72.0;
    private static final double Y_MIN_IN = -72.0;
    private static final double Y_MAX_IN =  72.0;

    private static boolean outOfBounds(double xIn, double yIn) {
        return xIn < X_MIN_IN || xIn > X_MAX_IN || yIn < Y_MIN_IN || yIn > Y_MAX_IN;
    }


    @FXML private Circle ballCircle;

    private Body ballBody;
    private BodyFixture mainFixture;

    private boolean airborne = false;
    private double airElapsedMs = 0;
    private double airTotalMs = 0;

    private double startXIn, startYIn;
    private double dirX, dirY;             // unit direction on field
    private double vHorizInPerSec;         // horizontal speed along dir
    private double vVertInPerSec;          // vertical (fake) speed
    private static final double G_IN_PER_SEC2 = 386.0; // 9.8 m/s^2 in inches/s^2


    public void initialize() {
        super.initialize();
    }

    @Override
    public void setUpBody() {

        // VISUAL size (what you see)
        double visualRadiusPx = BALL_RADIUS_INCHES * VirtualField.PIXELS_PER_INCH;

        // HITBOX size (what collides) - smaller than visual
        double hitboxRadiusIn = 1.9; // <-- change this (in inches)
        double hitboxRadiusPx = hitboxRadiusIn * VirtualField.PIXELS_PER_INCH;

        // Temporarily set the circle size to the hitbox size so createBody uses it
        ballCircle.setRadius(hitboxRadiusPx);

        elementBody = Dyn4jUtil.createBody(
                ballCircle,
                this,
                0,
                0,
                new FixtureData(BALL_FILTER, 1.0, 0.6, 0.2)
        );

        ballBody = elementBody;
        mainFixture = ballBody.getFixture(0);


        // Restore the visual size after the physics body is made
        ballCircle.setRadius(visualRadiusPx);

        ballBody.setLinearDamping(4.0);
        ballBody.setAngularDamping(4.0);
    }

    public void startAirShot(double dirX, double dirY, double speedInPerSec, double angleDeg) {
        if (ballBody == null) return;

        // Normalize direction
        double mag = Math.sqrt(dirX * dirX + dirY * dirY);
        if (mag < 1e-9) mag = 1e-9;
        this.dirX = dirX / mag;
        this.dirY = dirY / mag;

        // Starting position (inches)
        this.startXIn = getX() / VirtualField.PIXELS_PER_INCH;
        this.startYIn = getY() / VirtualField.PIXELS_PER_INCH;

        double angleRad = Math.toRadians(angleDeg);
        this.vHorizInPerSec = speedInPerSec * Math.cos(angleRad);
        this.vVertInPerSec  = speedInPerSec * Math.sin(angleRad);

        // Total airtime until "land" (t = 2*vVert/g)
        double tSec = (2.0 * vVertInPerSec) / G_IN_PER_SEC2;
        if (tSec < 0.05) tSec = 0.05; // avoid tiny time

        this.airElapsedMs = 0;
        this.airTotalMs = tSec * 1000.0;
        this.airborne = true;

        // Make physics ignore collisions while airborne
        stopMotion();
        if (mainFixture != null) mainFixture.setSensor(true);
    }


    public void setVelocityInchesPerSec(double vxIn, double vyIn) {
        if (elementBody == null) return;
        // inches/sec -> meters/sec
        double vx = vxIn * 0.0254;
        double vy = vyIn * 0.0254;
        elementBody.setLinearVelocity(vx, vy);
    }

    public void stopMotion() {
        if (elementBody == null) return;
        elementBody.setLinearVelocity(0, 0);
        elementBody.setAngularVelocity(0);
        elementBody.clearAccumulatedForce();
        elementBody.clearAccumulatedTorque();
    }


    public void setBallColor(Color color) {
        // Safe in case called before FXML initialize finishes
        if (ballCircle == null) return;

        // Ensure we touch JavaFX node on the FX thread
        if (Platform.isFxApplicationThread()) {
            ballCircle.setFill(color);
        } else {
            Platform.runLater(() -> ballCircle.setFill(color));
        }
    }



    @Override
    public void updateState(double millis) {

        // If airborne, manually move along a projectile arc (fake Z)
        if (airborne) {
            airElapsedMs += millis;
            double t = airElapsedMs / 1000.0;

            // horizontal distance along the shot direction
            double s = vHorizInPerSec * t;

            double newXIn = startXIn + dirX * s;
            double newYIn = startYIn + dirY * s;

            // Update body transform to follow the “air” path
            setLocationInches(newXIn, newYIn);
            // If it flies out of the arena, respawn it
            if (outOfBounds(newXIn, newYIn)) {
                airborne = false;
                if (mainFixture != null) mainFixture.setSensor(false);
                setLocationInches(RESPAWN_X_IN, RESPAWN_Y_IN);
                stopMotion();

                return;
            }



            // If landed, re-enable collisions and give it a little rollout
            if (airElapsedMs >= airTotalMs) {
                airborne = false;
                if (mainFixture != null) mainFixture.setSensor(false);

                // small rollout forward after landing (tune)
                double rollout = Math.max(10, vHorizInPerSec * 0.25);
                setVelocityInchesPerSec(dirX * rollout, dirY * rollout);
            }
        }

        // Pull position/rotation from physics body (meters → pixels)
        Transform tForm = ballBody.getTransform();
        x = tForm.getTranslationX() * VirtualField.PIXELS_PER_METER;
        y = tForm.getTranslationY() * VirtualField.PIXELS_PER_METER;

        double xIn = x / VirtualField.PIXELS_PER_INCH;
        double yIn = y / VirtualField.PIXELS_PER_INCH;

        if (outOfBounds(xIn, yIn)) {
            setLocationInches(RESPAWN_X_IN, RESPAWN_Y_IN);
            stopMotion();
        }


        headingRadians = tForm.getRotationAngle();
    }


    @Override
    public synchronized void updateDisplay() {
        super.updateDisplay();
    }
}
