# broken control hubs

## Symptoms

- code upload with Android Studio works
- `adb shell` works
  - can get root privileges just by using `su` (no password)
- web ports are unreachable or drop out every few seconds
  - 192.168.43.1:8080 (robot controller)
  - 192.168.43.1:8001 (Panels - depends on the robot controller, so it dies with it)
- the exact same code works fine on other hubs
- reflashing the firmware/OS and reinstalling the app does **not** fix it

## Cause

A stale **OnBot Java** build left on the hub's sdcard. The compiled jar lives in
`/sdcard/FIRST/java/build/jars/onBotJavaJar-*/` (`OnBotJava.jar` + `classes.dex`), and
`/sdcard/FIRST/java/status/currentOnBotJavaDir.txt` tells the app to load it. Loose `.class` files
may also be in `/sdcard/FIRST/java/build/classes/`, but deleting **only** those is not enough. The
app loads the jar.

Sinister (the class loader Panels uses) tries to load OnBot Java code on startup. It asks for a
leftover class (`teamcode/odometry/odometry`) by its path instead of its class name, Java rejects
it, and the error crashes the whole robot controller app.
Android restarts it, it crashes again, and so on, roughly every 45 seconds. Each crash takes down
the web server, which looks like the connection "dropping out".

Reflashing and reinstalling don't help because `/sdcard/FIRST` survives both. Other hubs work
because they don't have the leftover files.

### How to confirm

```sh
# process ID changes between runs = crash loop
adb shell pidof com.qualcomm.ftcrobotcontroller

# look for this in the crash log:
#   ClassNotFoundException ... at dev.frozenmilk.sinister ... OnBotLoadEventHandler
adb logcat -b crash -d

# any onBotJavaJar-* folder here is a suspect
adb shell ls -R /sdcard/FIRST/java/build

# see which classes a build contains
adb pull "/sdcard/FIRST/java/build/jars/<onBotJavaJar-folder>/OnBotJava.jar" && unzip -l OnBotJava.jar
```

## Fix

NOTE: this will remove all OnBotJava data!

1. Back up the robot configs (only the `.xml` files):
   ```sh
   adb shell 'ls /sdcard/FIRST/*.xml' | tr -d '\r' | xargs -I{} adb pull {} .
   ```

2. Delete the stale OnBot Java build (the jars are the important part) and its source:
   ```sh
   adb shell 'su 0 rm -rf /sdcard/FIRST/java/build/jars/* /sdcard/FIRST/java/build/classes/* /sdcard/FIRST/java/status/currentOnBotJavaDir.txt'
   adb shell rm -rf /sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode
   ```
   If nobody uses OnBot Java, removing all of `/sdcard/FIRST/java` is fine too. Otherwise any
   source left in `java/src` gets compiled back into a new jar the next time someone hits Build in
   the OnBot Java editor.

3. Restart the app (or reinstall from Android Studio):
   ```sh
   adb shell am force-stop com.qualcomm.ftcrobotcontroller
   adb shell monkey -p com.qualcomm.ftcrobotcontroller -c android.intent.category.LAUNCHER 1
   ```

4. Run `pidof` a few times over a couple of minutes. The process ID should no longer change.

To restore configs later, from the folder you pulled them into: `adb push *.xml /sdcard/FIRST/`

To open the robot controller pages over USB instead of Wi-Fi:
`adb forward tcp:8080 tcp:8080 && adb forward tcp:8001 tcp:8001`, then go to `http://localhost:8080`
(Panels at `http://localhost:8001`).

## Status

- **FTC-aW2L**: fixed with the steps above (clearing only `build/classes` was not enough. The jar in
  `build/jars` was the real culprit)
- **FTC-BVdZ**: probably has the same problem; also, it's XT-30 solder joint is loose causing random hard crashes
