package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.auto.paths.AutoPaths;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;

public class AllianceConfigTest {
    private static final double FIELD = 144;
    private static final double EPS = 1e-9;

    private static List<Field> poseFields(Class<?> c) {
        List<Field> out = new ArrayList<>();
        for (Field f : c.getDeclaredFields()) {
            if (Modifier.isStatic(f.getModifiers()) && f.getType() == Pose.class) out.add(f);
        }
        return out;
    }

    private static Pose get(Field f) throws IllegalAccessException {
        return (Pose) f.get(null);
    }

    private static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    // If this fails: a pose was added to or retuned on one alliance only, so Red drives a different route than Blue
    @Test
    public void redIsBlueRotatedHalfTurnAboutFieldCentre() throws Exception {
        List<Field> blue = poseFields(BlueAuto.class);
        assertEquals("Blue and Red must declare the same poses", blue.size(), poseFields(RedAuto.class).size());
        assertFalse(blue.isEmpty());
        for (Field b : blue) {
            Pose bp = get(b);
            Pose rp = get(RedAuto.class.getDeclaredField(b.getName()));
            String name = b.getName();
            assertEquals(name + " x", FIELD - bp.x(), rp.x(), EPS);
            assertEquals(name + " y", FIELD - bp.y(), rp.y(), EPS);
            assertEquals(name + " heading", 0, wrap(bp.heading() + Math.PI - rp.heading()), EPS);
        }
    }

    // If this fails: a pose is missing from buildConfig(), so a null pose crashes path building at init
    @Test
    public void everyPoseFieldReachesTheConfig() throws Exception {
        AutoConfig blue = new BlueAuto().buildConfig();
        int reached = 0;
        for (Field f : AutoConfig.class.getDeclaredFields()) {
            if (f.getType() != Pose.class) continue;
            assertNotNull(f.getName(), f.get(blue));
            reached++;
        }
        assertEquals(poseFields(BlueAuto.class).size(), reached);
    }

    // If this fails: a retuned pose makes Pedro throw while building paths (e.g. coincident points -> "cannot normalize
    // 0 vector"), which crashes the OpMode at init instead of on the bench
    @Test
    public void bothAlliancesBuildEveryPath() throws Exception {
        for (AutoConfig config : new AutoConfig[]{new BlueAuto().buildConfig(), new RedAuto().buildConfig()}) {
            AutoPaths paths = new AutoPaths(config);
            int built = 0;
            for (Field f : AutoPaths.class.getFields()) {
                if (f.getType() != Path.class) continue;
                assertNotNull(f.getName(), f.get(paths));
                built++;
            }
            assertTrue("expected the route's paths", built >= 15);
        }
    }
}
