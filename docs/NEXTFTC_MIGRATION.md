# TestRobot: NextFTC v2 migration

This project uses **NextFTC v2, artifact version 0.2.1**, with Java robot code. The v2 family currently publishes under `dev.nextftc.v2`; the artifact version is not `2.0.0`. Do not mix these dependencies or APIs with NextFTC v1 tutorials.

The implementation follows the current [project structure](https://nextftc.dev/robot/project-structure/), [NextRobot](https://nextftc.dev/robot/nextrobot/), [mechanism](https://nextftc.dev/robot/mechanisms/), [OpMode](https://nextftc.dev/robot/nextopmode/), and [gamepad](https://nextftc.dev/robot/command-gamepad/) guidance. Lifecycle details were also checked against the published `robot-0.2.1-sources.jar`, because the exact order of updates matters for the recorder.

## Dependencies and compatibility

- `TeamCode/build.gradle` declares `dev.nextftc.v2:control:0.2.1`, `hardware:0.2.1`, and `robot:0.2.1`. The robot module brings in Ivy 1.1.1 transitively. Java does not need the Kotlin Gradle plugin to consume these libraries.
- The Dairy Maven repository supplies the Sinister/Sloth runtime scanner required by NextFTC.
- All FTC SDK artifacts are aligned to **12.0.0**, and the app manifest identifies version 12.0, code 63, matching the official FTC release. Sloth 0.3.2 scans the SDK's `Utility` annotation, which does not exist in the former 11.1 dependencies. Merely compiling a Java app against 11.1 would not detect this runtime linkage problem.
- Six bundled AprilTag examples were refreshed from the official FTC `v12.0` tag because SDK 12 changes the detection types. Their FTC SDK lifecycles remain intact.
- The Android plugin remains 8.7.0 and the Gradle wrapper remains 8.9. D8/R8 is explicitly pinned to 9.1.31, and lint to 9.1.0, to read newer Kotlin dependencies. NextFTC uses Kotlin 2.3 and Sloth brings Kotlin 2.4.10. See [Android's Kotlin compatibility table](https://developer.android.com/build/kotlin-support) and [the documented lint override](https://developer.android.com/develop/ui/compose/tooling/lint).
- PedroPathing 2.1.2, Pedro telemetry, Panels, the hardware names, and the Pinpoint calibration remain as configured before the migration.

Use a matching FTC 12 Driver Station when deploying this Robot Controller build. The migration does not install either app or change the robot configuration.

## Where the code lives

All paths below are relative to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/testrobot/`.

| Location | Responsibility |
| --- | --- |
| `Robot.java` | The one public `NextRobot`; owns the mechanisms and per-OpMode resource setup. |
| `mechanisms/Drive.java` | Owns Pedro's follower, manual drive, aiming, drive commands, and follower updates. |
| `mechanisms/Intake.java` | Optional intake hardware and commands. |
| `commands/FollowPathCommand.java` | Drive ownership, arrival detection, timeout, and interruption behavior. |
| `commands/ReplayCommand.java` | One replay control iteration per scheduler tick. |
| `opmodes/base/RobotOpMode.java` | Initialization, STOP handling, pose saving, and guaranteed resource cleanup. |
| `opmodes/base/MainTeleOp.java` | Shared driver bindings for Main Driving and Recorder. |
| `opmodes/base/BaseAuto.java` | Starts a supplied autonomous command. |
| `opmodes/teleop/` | Main Driving and Recorder. |
| `opmodes/auto/` | Path Test, Drive To Center, and Replay. |
| `config/AutonomousConstants.java` | Configurable start/center poses and autonomous timeouts. |
| `utils/` | Recording format/parser, pose storage, RGB effect, field drawing, and tuning constants. |

`TestRobot`, `Robot.getInstance()`, `DriveController`, `IntakeController`, and the custom `Controls.Button`/`Trigger` polling implementation were replaced. NextFTC injects the same discovered `Robot` into each concrete OpMode through its public `OpModeName(Robot robot)` constructor. Each INIT rebuilds the follower and resets the drive state; constructors used by discovery do not access hardware.

The package prefix `testrobot` is retained. NextFTC's suggested layout is a convention, so there is no need to move unrelated code into its packages. The upstream Pedro `Tuning` menu still uses its `SelectableOpMode` lifecycle and the SDK samples remain SDK examples. Rewriting those library tools as NextFTC OpModes would change how their selection and tuning logic works.

## How a loop works

NextFTC 0.2.1 executes these steps during PLAY:

1. The OpMode's `periodic()` prepares its telemetry.
2. NextFTC calls the robot and mechanism `periodic()` methods. `Drive.periodic()` updates Pedro and draws the field.
3. A final observer mechanism calls the recorder after that hardware update.
4. NextFTC polls gamepad triggers and executes the Ivy scheduler. Drive commands prepare inputs for Pedro's next update.
5. NextFTC ticks its motor wrappers and flushes telemetry.

This order means that the recorder logs the drive inputs actually applied by the preceding scheduler tick alongside the resulting motor powers and pose. It does not mix a newly moved joystick with powers from an older cycle. No team OpMode calls `follower.update()` in its active loop, so the follower is updated once per active cycle.

In this release, NextFTC does not poll the scheduler or run mechanism periodic updates during INIT. The shared lifecycle hook updates localization there. TeleOp uses a separate INIT event loop for starting-pose selection and retires completed instant commands with `Scheduler.execute()`. Active driving bindings are installed only at PLAY.

Cleanup is in a custom `OpModeHook.afterEnd()` as well as normal `end()`, with an idempotent guard. The hook is called from NextFTC's `finally` block if initialization or a running command throws. It stops the drive and optional intake, closes the recorder, saves the measured pose, and drops per-OpMode references. A STOP during INIT is checked before scheduling any movement in `start()`.

## Commands and drive ownership

A mechanism is the owner of a part of the robot. A command is an operation that temporarily needs that mechanism. Calling `.requiring(drive)` tells Ivy that another command must not take the same drive at the same time.

In TeleOp, an infinite default command reads the joysticks. The center command takes drive ownership and keeps it while Pedro follows and then holds its destination. Circle schedules a manual command with the same requirement, which interrupts the center command. Pedro stops that path, manual mode starts, and the suspended default command resumes.

Settings such as the aiming target are changed with instant gamepad bindings. They do not launch a second hardware update loop. The manual-mode guards match the existing controls.

`FollowPathCommand` starts a path once, waits for arrival or a monotonic timeout, and stops Pedro when interrupted. Natural completion preserves endpoint hold so that the next path can start smoothly. Its injectable clock makes the timeout tests deterministic.

## Autonomous behavior

“Path Test” now expresses the twelve intake/launch legs as an Ivy sequential group. The coordinates, 0.8 path power, five-second per-leg failsafe, and 25-second parking deadline are preserved. When the parking deadline expires, it interrupts the current cycle and builds a parking path from the robot's measured position.

Normal parking also builds from the measured position. Its heading now uses the declared parking pose, **0 degrees**. The former normal-parking path incorrectly used the starting heading, 90 degrees; its deadline branch already used the parking pose's 0 degrees.

All team OpModes now save their measured stopping pose. Previously, Path Test always saved the planned parking pose, even when stopped early. Saved-pose TeleOp initialization and Drive To Center therefore use the actual last measured pose.

“TestRobot Drive To Center” still starts from the saved pose and holds center until STOP. “Replay” loads the CSV during INIT, starts from its first recorded pose, runs its command until the recorded duration is complete, and then stops.

## Driver controls

| Phase | Control | Action |
| --- | --- | --- |
| INIT | D-pad left/right | Select the original left/right start pose. |
| INIT | D-pad down | Load `lastPose.txt`. |
| PLAY | Left stick | Translate with the original signs, deadzone, and speed scaling. |
| PLAY | Right stick X | Turn, unless aiming is active. |
| PLAY | Cross / A | Drive to center when robot-centric and not aiming. |
| PLAY | Circle / B | Return to manual driving. |
| PLAY | Triangle / Y | Toggle field-centric driving while manual. |
| PLAY | Square / X | Toggle target aiming while manual. |
| PLAY | R3 | Switch blue/red target while aiming. |
| PLAY | L3 | Reset pose to the left/right start according to current field X. |

The RGB gamepad effect is retained. Field-centric mode, aiming, and the target selection reset at INIT, preventing a previous OpMode's settings from leaking into the next run.

The intake remains disabled, matching the original code. To use a fitted motor configured as `intake`, change `Robot.INTAKE_ENABLED` to `true`. Right bumper toggles forward intake; left bumper toggles reverse intake. These bindings replace the previously commented-out intake block. An enabled intake is explicitly stopped on exit.

## Recording and replay compatibility

The file is still `robot_recording.csv` in Android external storage, with the same header, column order, units, 20 ms regular recording interval, event samples, and drive-mode values. Existing valid 21-column files remain readable. Manual rows use their joystick feedforward; path/hold rows recover feedforward from the four motor-power columns. Field-centric and aiming states remain part of the recording.

Replay retains the existing translation/rotation gains, interpolation, voltage scaling, filtering, and power limits. Its PID state is now per replay command and resets when aiming state or target changes in either direction. Previously those values were static and the transition checks could miss a change.

The parser is independent of Android and rejects missing/empty files, incompatible headers or row lengths, invalid drive modes, backwards timestamps, and non-finite numeric values before a replay is scheduled. A bad recording produces INIT telemetry and no motion command. The migration does not rewrite existing files on the robot.

## Extending the robot

To add a mechanism, implement `Mechanism`, put its hardware and command factories in `mechanisms/`, and include it in `Robot.getMechanisms()`. Use `instant(...)` for one-shot changes and `infinite(...)` for continuously refreshed outputs. Both helpers declare the mechanism requirement for you. Keep per-OpMode hardware setup in `initialize()` and release/stop it through `Robot`.

For a new autonomous, extend `BaseAuto`, add `@NextAutonomous`, provide the public constructor taking `Robot`, and implement `startingPose()` and `autonomousCommand()`. Compose drive and mechanism commands with `Groups.sequential(...)` or `Groups.parallel(...)`; commands in parallel must not compete for the same mechanism.

For another driver mode, extend `MainTeleOp` if it shares these bindings. Extend `RobotOpMode` directly for a different control layout. Use `@NextTeleop` rather than the SDK's `@TeleOp` on a NextFTC OpMode.

## Verification and first hardware run

Run with JDK 17 or 21; this migration was checked with the installed JetBrains JDK 21. The project's Gradle 8.9 wrapper is not compatible with the machine's default JDK 25.

```sh
./gradlew :TeamCode:assembleDebug :TeamCode:testDebugUnitTest :TeamCode:lintDebug
```

There are 14 regression tests covering drive ownership, interruption before a manual override, resuming a suspended default, path sequencing, timeouts, deadline parking, command reuse, old recording rows, feedforward reconstruction, and malformed recordings.

Verification on 2026-09-29: debug APK assembly passed, all 14 tests passed, and Android lint completed with 0 errors and 14 warnings. Twelve warnings are repeated native-library 16 KB alignment findings in FTC/OpenFTC dependencies; two concern unused template resources. The Kotlin metadata errors from the old lint engine are resolved. The APK is `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`.

A desktop build cannot verify Android's runtime class scan, real motor directions, localization accuracy, or physical replay tracking. On the robot, confirm all five team OpModes appear, check INIT and STOP before PLAY, verify each driver binding, run a short record/replay, test both normal and deadline parking, and switch between OpModes repeatedly. Retain the Pedro Tuning menu for localization and follower checks.
