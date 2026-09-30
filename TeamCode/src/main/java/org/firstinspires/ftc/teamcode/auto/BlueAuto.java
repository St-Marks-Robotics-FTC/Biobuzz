/** base blue auto **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class BlueAuto extends BozoAuto { // these positions override the base auto class
    public static Pose pose1 = new Pose (0, 0, 0);
    public static Pose pose2 = new Pose (0, 0, 0);
    public static Pose pose3 = new Pose (0, 0, 0);

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                pose1,
                pose2,
                pose3
        );
    }
}
