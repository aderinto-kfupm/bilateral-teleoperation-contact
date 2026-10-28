#!/usr/bin/env python3
"""Inspect a PS5 DualSense controller and test its vibration motors."""

import argparse
import sys
import time


def print_devices(pygame) -> None:
    pygame.joystick.quit()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count == 0:
        print("No controllers found.")
        return

    for index in range(count):
        joystick = pygame.joystick.Joystick(index)
        joystick.init()
        print(
            f"[{index}] {joystick.get_name()} | "
            f"axes={joystick.get_numaxes()} "
            f"buttons={joystick.get_numbuttons()} "
            f"hats={joystick.get_numhats()}"
        )
        joystick.quit()


def rumble(joystick, strength: float, duration_ms: int) -> bool:
    try:
        supported = joystick.rumble(strength, strength, duration_ms)
    except (AttributeError, NotImplementedError):
        print("Rumble is not supported by this pygame/backend combination.")
        return False
    if not supported:
        print("The controller was detected, but rumble is not supported.")
        return False
    return True


def normalize_trigger(value: float) -> float:
    """Convert trigger axis values to a stable 0.0..1.0 strength range."""
    value = max(-1.0, min(1.0, float(value)))
    if value >= 0.0:
        return value
    return (value + 1.0) / 2.0


class F710Controller:
    """Runtime interface for reading a DualSense and triggering rumble."""

    def __init__(self, index: int = 0):
        import pygame

        self.pygame = pygame
        pygame.joystick.init()
        if index < 0 or index >= pygame.joystick.get_count():
            raise ValueError(f"Controller index {index} is not available.")
        self.joystick = pygame.joystick.Joystick(index)
        self.joystick.init()

    def read(self) -> tuple[list[float], list[bool]]:
        self.pygame.event.pump()
        for event in self.pygame.event.get():
            if event.type == self.pygame.JOYDEVICEREMOVED:
                raise RuntimeError("The controller was disconnected.")
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        buttons = [bool(self.joystick.get_button(i)) for i in range(self.joystick.get_numbuttons())]
        return axes, buttons

    def vibrate(self, strength: float = 0.5, duration_ms: int = 100) -> bool:
        self.pygame.event.pump()
        return rumble(self.joystick, strength, duration_ms)

    def close(self) -> None:
        self.joystick.quit()
        self.pygame.joystick.quit()


def run(args, pygame) -> int:
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No controllers found. Connect the DualSense and try again.", file=sys.stderr)
        return 1

    joystick = pygame.joystick.Joystick(args.index)
    joystick.init()
    print(f"Using [{args.index}] {joystick.get_name()}")
    print(
        f"axes={joystick.get_numaxes()} buttons={joystick.get_numbuttons()} "
        f"hats={joystick.get_numhats()}"
    )
    print("Move a control or press a button. Press Ctrl+C to stop.")

    if args.rumble_test:
        pygame.event.pump()
        rumble(joystick, args.rumble_strength, args.rumble_duration)

    last_axes = {}
    button_states = [False] * joystick.get_numbuttons()
    r2_locked = False
    last_r2 = None
    r2_lock_feedback_sent = False
    r2_lock_threshold = max(0.5, float(args.r2_threshold))
    start = time.monotonic()

    try:
        while args.duration <= 0 or time.monotonic() - start < args.duration:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    value = joystick.get_axis(event.axis)
                    if event.axis in (4, 5):
                        r2_value = normalize_trigger(value)
                        if last_r2 != r2_value:
                            last_r2 = r2_value
                            scaled_force = r2_value * args.r2_force_scale
                            print(f"R2 axis {event.axis}: {r2_value:.3f} (force={scaled_force:.3f})")
                            if args.r2_lock:
                                if r2_value >= r2_lock_threshold and not r2_locked:
                                    r2_locked = True
                                    r2_lock_feedback_sent = False
                                    print("R2 LOCKED")
                                elif r2_value < r2_lock_threshold and r2_locked:
                                    r2_locked = False
                                    r2_lock_feedback_sent = False
                                    print("R2 UNLOCKED")
                                if r2_locked and not r2_lock_feedback_sent:
                                    rumble(joystick, 1.0, 180)
                                    r2_lock_feedback_sent = True
                                elif not r2_locked and not r2_lock_feedback_sent:
                                    rumble(joystick, 1.0, 120)
                                    r2_lock_feedback_sent = True
                    if abs(value) >= args.deadzone or event.axis in last_axes:
                        if last_axes.get(event.axis) != value:
                            print(f"axis {event.axis}: {value:+.3f}")
                            last_axes[event.axis] = value
                elif event.type == pygame.JOYBUTTONDOWN:
                    continue
                elif event.type == pygame.JOYBUTTONUP:
                    continue
                elif event.type == pygame.JOYHATMOTION:
                    print(f"hat {event.hat}: {event.value}")
                elif event.type == pygame.JOYDEVICEADDED:
                    print(f"controller added: device={event.device_index}")
                elif event.type == pygame.JOYDEVICEREMOVED:
                    print(f"controller removed: instance={event.instance_id}")
            current_buttons = [
                bool(joystick.get_button(index))
                for index in range(joystick.get_numbuttons())
            ]
            for index, (was_pressed, is_pressed) in enumerate(
                zip(button_states, current_buttons)
            ):
                if is_pressed and not was_pressed:
                    print(f"button {index}: DOWN")
                    if args.rumble_on_button:
                        rumble(joystick, args.rumble_strength, args.rumble_duration)
                elif was_pressed and not is_pressed:
                    print(f"button {index}: UP")
            button_states = current_buttons
            time.sleep(args.poll_interval)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        joystick.quit()
        pygame.joystick.quit()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="List connected controllers and exit")
    parser.add_argument("--index", type=int, default=0, help="Controller index to use (default: 0)")
    parser.add_argument("--duration", type=float, default=0, help="Stop after this many seconds; 0 means until Ctrl+C")
    parser.add_argument("--poll-interval", type=float, default=0.01, help="Event polling interval in seconds")
    parser.add_argument("--deadzone", type=float, default=0.05, help="Ignore small axis changes below this value")
    parser.add_argument("--r2-threshold", type=float, default=0.25, help="R2 activation threshold from 0.0 to 1.0")
    parser.add_argument("--r2-force-scale", type=float, default=1.0, help="Scale factor applied to R2 strength for force output")
    parser.add_argument("--r2-lock", action="store_true", help="Lock an action when R2 exceeds the threshold")
    parser.add_argument("--rumble-test", action="store_true", help="Vibrate once when the script starts")
    parser.add_argument("--rumble-on-button", action="store_true", help="Vibrate when any button is pressed")
    parser.add_argument("--rumble-strength", type=float, default=0.5, help="Rumble strength from 0.0 to 1.0")
    parser.add_argument("--rumble-duration", type=int, default=150, help="Rumble duration in milliseconds")
    args = parser.parse_args()

    if not 0.0 <= args.rumble_strength <= 1.0:
        parser.error("--rumble-strength must be between 0.0 and 1.0")
    if not 0.0 <= args.r2_threshold <= 1.0:
        parser.error("--r2-threshold must be between 0.0 and 1.0")
    if not 0.0 <= args.r2_force_scale <= 10.0:
        parser.error("--r2-force-scale must be between 0.0 and 10.0")
    if args.poll_interval <= 0:
        parser.error("--poll-interval must be greater than 0")

    try:
        import pygame
    except ImportError:
        print("pygame is required. Install it with: python -m pip install pygame", file=sys.stderr)
        return 2

    pygame.init()
    try:
        if args.list:
            print_devices(pygame)
            return 0
        return run(args, pygame)
    finally:
        pygame.quit()


if __name__ == "__main__":
    raise SystemExit(main())