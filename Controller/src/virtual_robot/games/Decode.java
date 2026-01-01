package virtual_robot.games;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Polygon;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;

import virtual_robot.controller.VirtualGameElement;
import virtual_robot.game_elements.classes.Ball;
import javafx.scene.paint.Color;
import javafx.geometry.Rectangle2D;
import virtual_robot.controller.VirtualBot;

import virtual_robot.controller.Filters;
import virtual_robot.controller.Game;
import virtual_robot.controller.VirtualField;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.HashMap;
import java.util.Map;
import virtual_robot.config.Config;

/**
 * Decode is just like NoGame, except that there will be stationary (infinite mass) bodies
 * representing the Goals and Ramps
 */
public final class Decode extends Game {

    // ======= SCORING / BIN CONFIG (ALL IN INCHES) =======

    // Goal mouth sensors (INCHES) — placed at the actual hoop openings
    private static final Rectangle2D BLUE_HOOP_SENSOR = new Rectangle2D(-72, 50, 22, 22);
    private static final Rectangle2D RED_HOOP_SENSOR  = new Rectangle2D( 50, 50, 22, 22);

    // Extra catch zones near top corners so stuck balls still score
    // (Make these BIG on purpose; we only trigger after "stuck" timer anyway)
    private static final Rectangle2D BLUE_TOP_POCKET = new Rectangle2D(-72, 56, 50, 20);
    private static final Rectangle2D RED_TOP_POCKET  = new Rectangle2D( 22, 56, 50, 20);

    private static final double BALL_RADIUS_IN = 2.0;

    // Where balls “appear” after going through the hoop (top of the chute)
    private static final double BLUE_CHUTE_X = -63;
    private static final double BLUE_CHUTE_Y = 58;
    private static final double RED_CHUTE_X  =  63;
    private static final double RED_CHUTE_Y  = 58;

    // Where balls stack in the grey bin (9 spots)
    private static final double[][] BLUE_BIN_SPOTS = {
            {-70, 10}, {-66, 10}, {-62, 10},
            {-70,  6}, {-66,  6}, {-62,  6},
            {-70,  2}, {-66,  2}, {-62,  2},
    };

    private static final double[][] RED_BIN_SPOTS = {
            {70, 10}, {66, 10}, {62, 10},
            {70,  6}, {66,  6}, {62,  6},
            {70,  2}, {66,  2}, {62,  2},
    };

    // Overflow “drop out” spawn (end of grey area)
    private static final double BLUE_OVERFLOW_X = -56;
    private static final double BLUE_OVERFLOW_Y =  14;

    private static final double RED_OVERFLOW_X  =  56;
    private static final double RED_OVERFLOW_Y  =  14;

    // Hopper/bin areas (used to dump ALL balls in the bin/overflow when lever is pulled)
    private static final Rectangle2D BLUE_HOPPER_AREA = new Rectangle2D(-72, 0, 22, 18);
    private static final Rectangle2D RED_HOPPER_AREA  = new Rectangle2D( 50, 0, 22, 18);

    // Lever zones (robot touching this triggers release)
    private static final Rectangle2D BLUE_LEVER_ZONE = new Rectangle2D(-70, 0, 8, 10);
    private static final Rectangle2D RED_LEVER_ZONE  = new Rectangle2D( 63, 0, 8, 10);

    // Release exit locations (where balls drop out when lever opens)
    private static final double BLUE_RELEASE_X = -52;
    private static final double BLUE_RELEASE_Y =  16;

    private static final double RED_RELEASE_X  =  52;
    private static final double RED_RELEASE_Y  =  16;

    // ======= MATCH / PARKING =======
    private static final Rectangle2D BLUE_PARK_ZONE = new Rectangle2D(24, -46, 18, 18);
    private static final Rectangle2D RED_PARK_ZONE  = new Rectangle2D(-42, -46, 18, 18);
    private boolean matchEnded = false;

    // Scoring values
    private static final int GOAL_SCORE = 3;
    private static final int OVERFLOW_SCORE = 1; // any ball after 9 stored is 1 point
    private static final int PARK_SCORE = 10;

    // ======= INTERNAL STATE =======
    private final HashSet<Ball> blueBinBalls = new HashSet<>();
    private final HashSet<Ball> redBinBalls  = new HashSet<>();
    private final HashSet<Ball> scoredLock = new HashSet<>();

    private boolean blueLeverPrev = false;
    private boolean redLeverPrev  = false;
    private boolean parkingAwarded = false;

    // ======= STUCK DETECTOR =======
    // Track last position in inches + how long it hasn't moved
    private final Map<Ball, Double> stuckMs = new HashMap<>();
    private final Map<Ball, double[]> lastPosIn = new HashMap<>();

    private static final double STUCK_MOVE_EPS_IN = 0.25;   // if moved less than this, count as "not moved"
    private static final double STUCK_TIME_MS = 650;        // how long before we force-score it

    private static double pxToIn(double px) {
        return px / VirtualField.PIXELS_PER_INCH;
    }

    /** Overlap test: ball overlaps rect (not just center-in-rect). */
    private static boolean ballOverlapsRect(double ballXIn, double ballYIn, double ballRadiusIn, Rectangle2D r) {
        double closestX = Math.max(r.getMinX(), Math.min(ballXIn, r.getMaxX()));
        double closestY = Math.max(r.getMinY(), Math.min(ballYIn, r.getMaxY()));
        double dx = ballXIn - closestX;
        double dy = ballYIn - closestY;
        return (dx * dx + dy * dy) <= (ballRadiusIn * ballRadiusIn);
    }

    // Robot footprint for lever collision (mecanum_bot.fxml is 75px wide)
    private static final double BOT_SIZE_PX = 75.0;
    private static final double BOT_HALF_W_IN = (BOT_SIZE_PX / 2.0) / VirtualField.PIXELS_PER_INCH;
    private static final double BOT_HALF_H_IN = (BOT_SIZE_PX / 2.0) / VirtualField.PIXELS_PER_INCH;

    private static boolean botOverlapsZone(Rectangle2D zone, VirtualBot bot) {
        if (bot == null) return false;

        double botXIn = pxToIn(bot.getX());
        double botYIn = pxToIn(bot.getY());

        double botMinX = botXIn - BOT_HALF_W_IN;
        double botMaxX = botXIn + BOT_HALF_W_IN;
        double botMinY = botYIn - BOT_HALF_H_IN;
        double botMaxY = botYIn + BOT_HALF_H_IN;

        double zMinX = zone.getMinX();
        double zMaxX = zone.getMaxX();
        double zMinY = zone.getMinY();
        double zMaxY = zone.getMaxY();

        return botMaxX >= zMinX && botMinX <= zMaxX && botMaxY >= zMinY && botMinY <= zMaxY;
    }

    private void drawDebugRect(Rectangle2D r, Color color) {
        if (controller == null) return;

        double halfFieldPx = Config.FIELD_WIDTH / 2.0;

        double xPx = halfFieldPx + r.getMinX() * VirtualField.PIXELS_PER_INCH;
        double yPx = halfFieldPx - (r.getMinY() + r.getHeight()) * VirtualField.PIXELS_PER_INCH;

        javafx.scene.shape.Rectangle box = new javafx.scene.shape.Rectangle(
                xPx,
                yPx,
                r.getWidth() * VirtualField.PIXELS_PER_INCH,
                r.getHeight() * VirtualField.PIXELS_PER_INCH
        );

        box.setFill(Color.color(color.getRed(), color.getGreen(), color.getBlue(), 0.18));
        box.setStroke(color);
        box.setStrokeWidth(2);

        javafx.application.Platform.runLater(() ->
                controller.getFieldPane().getChildren().add(box)
        );
    }

    public void checkForMatchEnd(double millisRemaining) {
        if (matchEnded) return;
        if (millisRemaining <= 0) {
            matchEnded = true;
            onMatchEnd();
        }
    }

    private void addScoreSafe(int delta) {
        if (controller == null) return;
        try {
            Method m = controller.getClass().getMethod("addScore", int.class);
            m.invoke(controller, delta);
        } catch (Exception ignored) { }
    }

    private void sendToBin(Ball b, boolean toBlue) {
        b.stopMotion();
        b.setOnField(false);

        if (toBlue) b.setLocationInches(BLUE_CHUTE_X, BLUE_CHUTE_Y);
        else        b.setLocationInches(RED_CHUTE_X,  RED_CHUTE_Y);

        b.setOnField(true);
        b.setVelocityInchesPerSec(0, -25);

        if (toBlue) {
            if (blueBinBalls.size() >= 9) {
                b.setLocationInches(BLUE_OVERFLOW_X, BLUE_OVERFLOW_Y);
                b.setOnField(true);
                b.setVelocityInchesPerSec(0, -25);
            } else {
                int idx = blueBinBalls.size();
                b.setLocationInches(BLUE_BIN_SPOTS[idx][0], BLUE_BIN_SPOTS[idx][1]);
                blueBinBalls.add(b);
                b.setOnField(true);
                b.stopMotion();
            }
        } else {
            if (redBinBalls.size() >= 9) {
                b.setLocationInches(RED_OVERFLOW_X, RED_OVERFLOW_Y);
                b.setOnField(true);
                b.setVelocityInchesPerSec(0, -25);
            } else {
                int idx = redBinBalls.size();
                b.setLocationInches(RED_BIN_SPOTS[idx][0], RED_BIN_SPOTS[idx][1]);
                redBinBalls.add(b);
                b.setOnField(true);
                b.stopMotion();
            }
        }
    }

    /** Lever release: dump ALL balls in hopper area (includes 9 stored + any overflow balls). */
    private void releaseBin(boolean blue) {
        Rectangle2D hopper = blue ? BLUE_HOPPER_AREA : RED_HOPPER_AREA;

        ArrayList<Ball> toRelease = new ArrayList<>();
        for (Ball b : Ball.balls) {
            if (b == null || !b.isOnField()) continue;

            double xIn = pxToIn(b.getX());
            double yIn = pxToIn(b.getY());

            if (ballOverlapsRect(xIn, yIn, BALL_RADIUS_IN, hopper)) {
                toRelease.add(b);
            }
        }

        for (Ball b : toRelease) {
            b.stopMotion();

            if (blue) {
                b.setLocationInches(BLUE_RELEASE_X, BLUE_RELEASE_Y);
                b.setOnField(true);
                b.setVelocityInchesPerSec(0, -30);
                blueBinBalls.remove(b);
            } else {
                b.setLocationInches(RED_RELEASE_X, RED_RELEASE_Y);
                b.setOnField(true);
                b.setVelocityInchesPerSec(0, -30);
                redBinBalls.remove(b);
            }

            scoredLock.remove(b);
            stuckMs.remove(b);
            lastPosIn.remove(b);
        }
    }

    public void onMatchEnd() {
        if (parkingAwarded) return;
        parkingAwarded = true;

        if (controller == null) return;
        VirtualBot bot = controller.getBot();
        if (bot == null) return;

        double botXIn = pxToIn(bot.getX());
        double botYIn = pxToIn(bot.getY());

        boolean inBlue = BLUE_PARK_ZONE.contains(botXIn, botYIn);
        boolean inRed  = RED_PARK_ZONE.contains(botXIn, botYIn);

        if (inBlue || inRed) addScoreSafe(PARK_SCORE);
    }

    /** If a ball is in a top pocket and hasn't moved for STUCK_TIME_MS, force-score it and send to bin. */
    private void handleStuckTopPocket(Ball b, double xIn, double yIn, double millis) {
        boolean inBluePocket = BLUE_TOP_POCKET.contains(xIn, yIn);
        boolean inRedPocket  = RED_TOP_POCKET.contains(xIn, yIn);

        if (!inBluePocket && !inRedPocket) {
            stuckMs.remove(b);
            lastPosIn.remove(b);
            return;
        }

        double[] last = lastPosIn.get(b);
        if (last == null) {
            lastPosIn.put(b, new double[]{xIn, yIn});
            stuckMs.put(b, 0.0);
            return;
        }

        double dx = xIn - last[0];
        double dy = yIn - last[1];
        double dist = Math.sqrt(dx * dx + dy * dy);

        // update last position always
        last[0] = xIn;
        last[1] = yIn;

        double acc = stuckMs.getOrDefault(b, 0.0);

        if (dist <= STUCK_MOVE_EPS_IN) {
            acc += millis;
            stuckMs.put(b, acc);
        } else {
            // moved -> reset timer
            stuckMs.put(b, 0.0);
            return;
        }

        if (acc >= STUCK_TIME_MS) {
            // Force it to count + bin it
            if (scoredLock.contains(b)) return;

            if (inBluePocket) {
                scoredLock.add(b);
                int pts = (blueBinBalls.size() >= 9) ? OVERFLOW_SCORE : GOAL_SCORE;
                addScoreSafe(pts);
                sendToBin(b, true);
            } else if (inRedPocket) {
                scoredLock.add(b);
                int pts = (redBinBalls.size() >= 9) ? OVERFLOW_SCORE : GOAL_SCORE;
                addScoreSafe(pts);
                sendToBin(b, false);
            }

            stuckMs.remove(b);
            lastPosIn.remove(b);
        }
    }

    @Override
    public void updateGameElementState(double millis) {
        super.updateGameElementState(millis);

        // 1) Scoring + stuck handling
        for (Ball b : Ball.balls) {
            if (b == null || !b.isOnField()) continue;

            double xIn = pxToIn(b.getX());
            double yIn = pxToIn(b.getY());

            // If ball is stuck in top pocket, force-score after timer
            handleStuckTopPocket(b, xIn, yIn, millis);

            // normal hoop scoring (skip if already scored recently or parked)
            if (scoredLock.contains(b)) continue;
            if (blueBinBalls.contains(b) || redBinBalls.contains(b)) continue;

            boolean inBlueGoal = ballOverlapsRect(xIn, yIn, BALL_RADIUS_IN, BLUE_HOOP_SENSOR);
            boolean inRedGoal  = ballOverlapsRect(xIn, yIn, BALL_RADIUS_IN, RED_HOOP_SENSOR);

            if (inBlueGoal) {
                scoredLock.add(b);
                int pts = (blueBinBalls.size() >= 9) ? OVERFLOW_SCORE : GOAL_SCORE;
                addScoreSafe(pts);
                sendToBin(b, true);

            } else if (inRedGoal) {
                scoredLock.add(b);
                int pts = (redBinBalls.size() >= 9) ? OVERFLOW_SCORE : GOAL_SCORE;
                addScoreSafe(pts);
                sendToBin(b, false);
            }

            checkForMatchEnd(Config.remainingMatchTimeMillis);
        }

        // 2) Lever detection (overlap)
        if (controller != null) {
            VirtualBot bot = controller.getBot();

            boolean blueLeverNow = botOverlapsZone(BLUE_LEVER_ZONE, bot);
            boolean redLeverNow  = botOverlapsZone(RED_LEVER_ZONE,  bot);

            if (blueLeverNow && !blueLeverPrev) releaseBin(true);
            if (redLeverNow  && !redLeverPrev)  releaseBin(false);

            blueLeverPrev = blueLeverNow;
            redLeverPrev  = redLeverNow;
        }
    }

    @Override
    public final void initialize() {
        super.initialize();

        Polygon blueGoalPoly = new Polygon(new Vector2(0, 0), new Vector2(0, -0.547),
                new Vector2(0.217, -0.547), new Vector2(0.604, 0));
        Rectangle blueRampRect = new Rectangle(0.217, 1.282);
        Polygon redGoalPoly = new Polygon(new Vector2(0, 0), new Vector2(-0.624, 0),
                new Vector2(-0.217, -0.547), new Vector2(0, -0.547));
        Rectangle redRampRect = new Rectangle(0.217, 1.282);

        // Debug overlays (draw ONCE)
        drawDebugRect(BLUE_PARK_ZONE, Color.DODGERBLUE);
        drawDebugRect(RED_PARK_ZONE, Color.RED);
        drawDebugRect(BLUE_HOOP_SENSOR, Color.CYAN);
        drawDebugRect(RED_HOOP_SENSOR, Color.ORANGERED);
        drawDebugRect(BLUE_LEVER_ZONE, Color.DEEPSKYBLUE);
        drawDebugRect(RED_LEVER_ZONE, Color.CRIMSON);
        drawDebugRect(BLUE_HOPPER_AREA, Color.LIGHTBLUE);
        drawDebugRect(RED_HOPPER_AREA, Color.PINK);

        // Show top pocket catch zones
        drawDebugRect(BLUE_TOP_POCKET, Color.YELLOW);
        drawDebugRect(RED_TOP_POCKET, Color.YELLOW);

        blueGoalPoly.translate(-VirtualField.HALF_FIELD_WIDTH_METERS, VirtualField.HALF_FIELD_WIDTH_METERS);
        blueRampRect.translate(-VirtualField.HALF_FIELD_WIDTH_METERS + blueRampRect.getWidth() / 2.0,
                blueRampRect.getHeight() / 2.0);
        redGoalPoly.translate(VirtualField.HALF_FIELD_WIDTH_METERS, VirtualField.HALF_FIELD_WIDTH_METERS);
        redRampRect.translate(VirtualField.HALF_FIELD_WIDTH_METERS - redRampRect.getWidth() / 2.0,
                redRampRect.getHeight() / 2.0);

        Ball.balls.clear();
        for (VirtualGameElement e : gameElements) {
            if (e instanceof Ball) {
                Ball.balls.add((Ball) e);
                Convex[] convexes = new Convex[]{blueGoalPoly, blueRampRect, redGoalPoly, redRampRect};
                for (Convex convex : convexes) {
                    Body body = new Body();
                    BodyFixture fixture = body.addFixture(convex);
                    fixture.setFilter(Filters.WALL_FILTER);
                    body.setMass(MassType.INFINITE);
                    world.addBody(body);
                }
            }
        }
    }

    @Override
    public void resetGameElements() {
        scoredLock.clear();
        blueBinBalls.clear();
        redBinBalls.clear();
        blueLeverPrev = false;
        redLeverPrev = false;
        parkingAwarded = false;

        stuckMs.clear();
        lastPosIn.clear();

        double[][] POS = new double[][]{
                { 43,  12},
                { 48,  12},
                { 53,  12},
                { 43, -12},
                { 48, -12},
                { 53, -12},
                { 43, -36},
                { 48, -36},
                { 53, -36},
                {-43,  12},
                {-48,  12},
                {-53,  12},
                {-43, -12},
                {-48, -12},
                {-53, -12},
                {-43, -36},
                {-48, -36},
                {-53, -36},
                { 69, -69},
                { 69, -64},
                { 69, -59},
                {-69, -69},
                {-69, -64},
                {-69, -59}
        };

        Color[] COLORS = new Color[]{
                Color.MEDIUMPURPLE,
                Color.MEDIUMPURPLE,
                Color.LIMEGREEN,
                Color.MEDIUMPURPLE,
                Color.LIMEGREEN,
                Color.MEDIUMPURPLE,
                Color.LIMEGREEN,
                Color.MEDIUMPURPLE,
                Color.MEDIUMPURPLE,
                Color.MEDIUMPURPLE,
                Color.MEDIUMPURPLE,
                Color.LIMEGREEN,
                Color.MEDIUMPURPLE,
                Color.LIMEGREEN,
                Color.MEDIUMPURPLE,
                Color.LIMEGREEN,
                Color.MEDIUMPURPLE,
                Color.MEDIUMPURPLE,
                Color.MEDIUMPURPLE,
                Color.LIMEGREEN,
                Color.MEDIUMPURPLE,
                Color.MEDIUMPURPLE,
                Color.LIMEGREEN,
                Color.MEDIUMPURPLE
        };

        for (int i = 0; i < Ball.balls.size(); i++) {
            Ball b = Ball.balls.get(i);
            int idx = i % POS.length;

            b.setLocationInches(POS[idx][0], POS[idx][1]);
            b.setBallColor(COLORS[idx]);
            b.setOnField(true);
        }

        updateDisplay();
    }

    @Override
    public boolean hasHumanPlayer() { return false; }
    @Override
    public final boolean isHumanPlayerAuto() { return false; }
    @Override
    public final void setHumanPlayerAuto(boolean selected) { }
    @Override
    public final void updateHumanPlayerState(double millis) { }
    @Override
    public final void requestHumanPlayerAction() { }
    @Override
    public final boolean isHumanPlayerActionRequested() { return false; }
    @Override
    public final void stopGameElements() { }
}
