# Keyboard teleoperation

The control firmware already installed on the robot supports this panel. No firmware update is needed. The local Python server uses the XDS110 serial link; the browser and server run only on this Mac.

## Open it

Double-click `Start-Teleop.command` at the repository root. It opens a local browser panel. Keep its Terminal window running; Ctrl-C stops the robot and closes the server. Close the existing teleop server before starting another copy, because each server exclusively owns the serial port.

From Terminal, the equivalent is:

```sh
.venv/bin/python tools/robot_teleop.py
```

It automatically selects the single XDS110 application UART. If needed, specify `--port /dev/cu.usbmodemM43210051`. Default speed is the physically tested 20%; `--speed` accepts 1–25. The local URL uses an available port and is printed at launch.

## Drive

1. Initially keep wheels supported clear to check directions, with the removed 5V jumper and disconnected servos as previously verified. Keep the USB cable slack.
2. Click **Enable controls**, then press and release **S1** on the LaunchPad.
3. After “Ready to drive,” press a fresh direction key and hold it. Release to stop.

| Key | Command |
|---|---|
| W / up arrow | Both wheels forward |
| S / down arrow | Both wheels backward |
| A / left arrow | Left wheel reverse, right forward (pivot left) |
| D / right arrow | Left wheel forward, right reverse (pivot right) |
| Forward + left/right | Arc with the inner wheel stopped |
| Space / Escape / Stop button | Stop and disarm; Enable + S1 required again |

Direction names follow the documented motor mapping. Confirm the actual chassis response before ground driving. This is manual wheel control; it does not enable the uncalibrated arm or encoder navigation.

Releasing direction keys stops wheel commands while leaving the session armed. Changing tabs/windows, hiding/closing the page, losing browser updates, or pressing S2/a bump stops and disarms. A key held before arming is discarded: press it again after the ready indication. Only the active panel can command movement; another panel must stop the current session before taking over.

## Timing and disconnect behavior

Browser key state is refreshed every 80 ms while the panel is focused. The server refuses stale/out-of-order movement, uses a 350 ms browser timeout, and renews 200 ms firmware PWM leases at roughly 100 ms intervals. It continues the firmware heartbeat while enabled. If the host stops sending serial commands, the motor lease expires independently; the firmware also has its existing 500 ms heartbeat watchdog. These are software timing bounds subject to scheduling/serial latency, not a guarantee of instantaneous mechanical braking.

The installed firmware briefly disables outputs while accepting each new PWM command; this can make low-speed teleop slightly pulsed. No reflashing was necessary. Stopping uses the driver's disabled/coast behavior. Space, S2 and the power switch remain available. Do not halt the MCU debugger with motor power applied.

USB errors stop the session and require restarting the launcher after reconnecting; there is no automatic motion resumption. The server binds only to `127.0.0.1`, checks the request origin/host and a session token, and never publishes robot controls onto the network.

## Tests

- `python3 -m unittest discover -s tests -v`: 30 tests pass, including 14 new teleop backend/HTTP tests.
- `node tests/test_teleop_ui.cjs`: mocked-DOM tests pass for key combinations, release, Space, focus/visibility loss, repeat/modifier handling, generation changes and stale responses.
- Live browser page and XDS110 connection verified against the installed control firmware. Startup leaves the robot disarmed. Firmware was not overwritten for teleop.

The page's key-release and page-lifecycle handling follows the browser events documented by [MDN keyup](https://developer.mozilla.org/en-US/docs/Web/API/Element/keyup_event) and [MDN visibilitychange](https://developer.mozilla.org/en-US/docs/Web/API/Document/visibilitychange_event). Server-side command leases provide the fallback if a browser event is lost.

On 2026-11-06 the user confirmed **WASD/arrow-key movement and release-to-stop work on the physical robot**. The session was subsequently stopped/disarmed for arm setup.
