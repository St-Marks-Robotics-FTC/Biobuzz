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

Stale **OnBot Java** classes left on the hub's sdcard in
`/sdcard/FIRST/java/build/classes/org/firstinspires/ftc/teamcode/`.

Sinister (the class loader Panels uses) tries to load OnBot Java code on startup. A leftover
class (`teamcode/odometry/odometry`) fails to load, which crashes the whole robot controller app.
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

# anything listed here is a suspect
adb shell ls -R /sdcard/FIRST/java/build/classes
```

## Fix

NOTE: this will remove all OnBotJava data!

1. Back up the robot configs (only the `.xml` files):
   ```sh
   adb shell 'ls /sdcard/FIRST/*.xml' | tr -d '\r' | xargs -I{} adb pull {} .
   ```

2. Delete the stale OnBot Java files:
   ```sh
   adb shell rm -rf /sdcard/FIRST/java/build/gensrc/classes/org/firstinspires/ftc/teamcode/
   adb shell rm -rf /sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode
   ```
   If nobody uses OnBot Java, removing all of `/sdcard/FIRST/java` is fine too. Otherwise any
   source left in `java/src` gets compiled back into `build/classes` the next time someone hits
   Build in the OnBot Java editor.

3. Reinstall from Android Studio.

To restore configs later: `adb push hub-configs/*.xml /sdcard/FIRST/`

## Status

- **FTC-aW2L**: fixed with the steps above
- **FTC-BVdZ**: not checked yet; probably the same cause. Check `/sdcard/FIRST/java/build/classes`
