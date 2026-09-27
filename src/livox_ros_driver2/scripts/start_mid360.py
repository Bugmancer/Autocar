#!/usr/bin/env python3
"""ROS launch prefix: singleton guard -> root network helper -> unprivileged driver."""

import argparse
import json
import logging
import os
from pathlib import Path
import socket
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mid360_network import HOST, LIDAR, Network, NetworkError

LOG = logging.getLogger("mid360_startup")
LOCK_NAME = "\0autocar_v1.mid360.driver"


def acquire_lock():
    # Linux abstract socket: shared across users, no writable lock file/symlink.
    guard = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        guard.bind(LOCK_NAME)
    except OSError as error:
        guard.close()
        raise NetworkError("Another guarded MID360 driver is starting/running") from error
    guard.set_inheritable(True)
    return guard


def check_existing_driver(proc_root=Path("/proc")):
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry / "cmdline").read_bytes().split(b"\0")[0].decode(errors="replace")
        except FileNotFoundError:
            continue
        except PermissionError as error:
            raise NetworkError("Cannot inspect PID %s to exclude another driver" % entry.name) from error
        if Path(command).name in ("livox_ros_driver2_node", "livox_ros_driver_node"):
            raise NetworkError("Existing Livox driver PID %s; stop it before starting this launch"
                               % entry.name)


def validate_config(path):
    with open(path, encoding="utf-8") as stream:
        config = json.load(stream)
    host = config.get("MID360", {}).get("host_net_info", {})
    keys = ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip")
    devices = config.get("lidar_configs", [])
    if (any(host.get(key) != HOST for key in keys) or
            len(devices) != 1 or devices[0].get("ip") != LIDAR):
        raise NetworkError("Config must select one MID360 at %s and host receive IP %s" % (LIDAR, HOST))


def start(config, command, takeover_video_address=False, prepare_only=False):
    if not prepare_only and (not command or Path(command[0]).name != "livox_ros_driver2_node"):
        raise NetworkError("Expected livox_ros_driver2_node executable after --")
    if os.geteuid() == 0:
        raise NetworkError("Run roslaunch as the normal user; only the network helper uses sudo")
    validate_config(config)
    guard = acquire_lock()
    try:
        check_existing_driver()
        network = Network()
        addresses, routes, transfer = network.preflight(takeover_video_address)
        configured = (not transfer and len(addresses) == 1 and
                      addresses[0]["prefixlen"] == 24 and bool(routes))
        if prepare_only or not configured:
            helper = str(Path(__file__).resolve().with_name("mid360_network.py"))
            helper_args = ["--takeover-video-address"] if takeover_video_address else []
            # roslaunch uses setsid: its children cannot reuse terminal sudo tickets.
            # Only --prepare-only, before roslaunch, may prompt on the caller's TTY.
            sudo_args = [] if prepare_only else ["-n"]
            result = subprocess.run(
                ["/usr/bin/sudo", *sudo_args, "/usr/bin/python3", helper, "--apply", *helper_args],
                capture_output=True, text=True, timeout=120 if prepare_only else 30)
            for line in (result.stdout + result.stderr).splitlines():
                LOG.info("network: %s", line)
            if result.returncode:
                raise NetworkError("Network helper failed. Check the error above; "
                                   "use bash scripts/with_mid360_network.sh to authenticate "
                                   "before roslaunch. Never sudo roslaunch.")
        else:
            LOG.info("Network already configured; verifying without privileged changes")
        check_existing_driver()
        network.verify_ready()
        if prepare_only:
            LOG.info("Network prepared; roslaunch may now start the guarded driver")
            return
        LOG.info("Network ready; executing %s (singleton guard retained until exit)", command[0])
        os.execv(command[0], command)
    finally:
        guard.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--takeover-video-address", action="store_true")
    parser.add_argument("--prepare-only", action="store_true",
                        help="Initialize from an interactive terminal before roslaunch; no driver")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    log_dir = Path(os.environ.get("ROS_HOME", str(Path.home() / ".ros"))) / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s",
                        handlers=[logging.StreamHandler(),
                                  logging.FileHandler(str(log_dir / "mid360_startup.log"))])
    try:
        command = args.command[1:] if args.command[:1] == ["--"] else args.command
        start(args.config, command, args.takeover_video_address, args.prepare_only)
    except (NetworkError, OSError, ValueError, subprocess.SubprocessError) as error:
        LOG.error("MID360 startup FAILED; driver not started: %s", error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
