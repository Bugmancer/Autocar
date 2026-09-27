#!/usr/bin/env python3
"""Initialize the MID360 link, optionally taking its address from the video NIC."""

import argparse
import json
import logging
import os
import shutil
import subprocess
import time


INTERFACE = "eno1"
VIDEO_INTERFACE = "enp100s0"
HOST = "192.168.1.50"
PREFIX = HOST + "/24"
LIDAR = "192.168.1.118"
LOG = logging.getLogger("mid360_network")


class NetworkError(RuntimeError):
    pass


class Network:
    def __init__(self):
        self.ip = shutil.which("ip", path="/usr/sbin:/usr/bin:/sbin:/bin")
        self.ping = shutil.which("ping", path="/usr/sbin:/usr/bin:/sbin:/bin")
        if not self.ip or not self.ping:
            raise NetworkError("System commands ip and ping are required")

    @staticmethod
    def run(command, check=True):
        LOG.info("%s", " ".join(command))
        result = subprocess.run(command, capture_output=True, text=True, timeout=5)
        if check and result.returncode:
            raise NetworkError("Command failed: %s: %s" %
                               (" ".join(command), result.stderr.strip()))
        return result

    def query(self, *args):
        # `ip -4 address show` hides NICs that do not have an IPv4 address yet.
        family = [] if args[0] == "address" else ["-4"]
        return json.loads(self.run([self.ip, "-j", *family, *args]).stdout)

    def change(self, *args):
        self.run([self.ip, "-4", *args])

    def preflight(self, takeover_video_address=False):
        interfaces = self.query("address", "show")
        target = None
        transfer = False
        for interface in interfaces:
            name = interface["ifname"]
            if name == INTERFACE:
                target = interface
            for address in interface.get("addr_info", []):
                if address.get("local") == LIDAR:
                    raise NetworkError("Lidar IP %s is assigned locally on %s" % (LIDAR, name))
                if name != INTERFACE and address.get("local") == HOST:
                    if name != VIDEO_INTERFACE or not takeover_video_address:
                        raise NetworkError(
                            "%s is occupied by %s; no network changes made. "
                            "Taking the video link address from enp100s0 requires "
                            "--takeover-video-address and interrupts video communication."
                            % (HOST, name))
                    source_addresses = [a for a in interface.get("addr_info", [])
                                        if a.get("family") == "inet"]
                    # Deleting a primary address can implicitly delete secondary ones.
                    if (len(source_addresses) != 1 or address.get("prefixlen") != 24 or
                            address.get("dynamic", False)):
                        raise NetworkError("enp100s0 must have only the static %s IPv4 address; "
                                           "refusing to disrupt other/DHCP addresses" % PREFIX)
                    transfer = True
        if target is None:
            raise NetworkError("Interface eno1 does not exist")
        if not {"UP", "LOWER_UP"}.issubset(target.get("flags", [])):
            raise NetworkError("eno1 is down or has no carrier; check cable/power/link first")
        addresses = [a for a in target.get("addr_info", []) if a.get("family") == "inet"]
        for address in addresses:
            if address["local"] != HOST or address.get("dynamic", False):
                raise NetworkError("eno1 has another/DHCP IPv4 address; refusing to disrupt it: %s/%s"
                                   % (address["local"], address["prefixlen"]))
        for route in self.query("route", "show", "default"):
            if route.get("dev") == INTERFACE or (
                    transfer and route.get("dev") == VIDEO_INTERFACE):
                raise NetworkError("%s carries a default route; refusing to reconfigure a shared link"
                                   % route["dev"])
        routes = self.query("route", "show", "exact", LIDAR + "/32")
        if len(routes) > 1 or any(
                r.get("dev") != INTERFACE or r.get("gateway") or
                r.get("type", "unicast") != "unicast" or
                r.get("prefsrc", HOST) != HOST for r in routes):
            raise NetworkError("Conflicting lidar /32 route; inspect it manually: %s" % routes)
        if transfer:
            LOG.warning("Authorized transfer pending: %s from enp100s0 to eno1; video will stop",
                        PREFIX)
        LOG.info("Preflight passed: eno1 has carrier; address conflicts checked")
        return addresses, routes, transfer

    def verify_route(self):
        # Check both source-bound traffic (SDK) and normal destination routing.
        for suffix in ((), ("from", HOST)):
            routes = self.query("route", "get", LIDAR, *suffix)
            if (len(routes) != 1 or routes[0].get("dev") != INTERFACE or
                    routes[0].get("gateway") or
                    routes[0].get("prefsrc", routes[0].get("from", routes[0].get("src"))) != HOST):
                raise NetworkError("Lidar route is not direct eno1 with source %s: %s" % (HOST, routes))

    def verify_neighbor(self, clear_cache=True):
        # Remove only this neighbor so a cached STALE/PERMANENT entry cannot pass.
        if clear_cache and self.query("neigh", "show", "to", LIDAR, "dev", INTERFACE):
            self.change("neigh", "del", LIDAR, "dev", INTERFACE)
        for _ in range(5):
            # ICMP may be filtered. A freshly resolved ARP neighbor is sufficient.
            self.run([self.ping, "-n", "-I", INTERFACE, "-c", "1", "-W", "1", LIDAR],
                     check=False)
            neighbors = self.query("neigh", "show", "to", LIDAR, "dev", INTERFACE)
            if any(n.get("lladdr") and "REACHABLE" in n.get("state", []) for n in neighbors):
                LOG.info("ARP reachable: %s on eno1", LIDAR)
                return
            time.sleep(0.2)
        raise NetworkError("ARP did not resolve %s on eno1; check radar power/IP/cable" % LIDAR)

    def verify_ready(self):
        addresses, routes, _ = self.preflight()
        if len(addresses) != 1 or addresses[0]["prefixlen"] != 24 or not routes:
            raise NetworkError("MID360 network needs initialization before starting the driver")
        self.verify_route()
        # The root helper already cleared cached ARP. Recheck without root writes.
        self.verify_neighbor(clear_cache=False)
        self.verify_route()

    def apply(self, takeover_video_address=False):
        addresses, routes, transfer = self.preflight(takeover_video_address)
        if os.geteuid() != 0:
            raise NetworkError("Network initialization requires root (use sudo for this helper only)")
        if transfer:
            LOG.warning("Moving only %s from enp100s0 to eno1; not restored automatically on exit",
                        PREFIX)
            self.change("address", "del", PREFIX, "dev", VIDEO_INTERFACE)
            # Strict check after transfer also catches a service immediately re-adding it.
            addresses, routes, _ = self.preflight()
        for address in addresses:
            if address["prefixlen"] != 24:
                self.change("address", "del", HOST + "/" + str(address["prefixlen"]),
                            "dev", INTERFACE)
        # Removing a primary address can also remove Linux secondary addresses.
        addresses, routes, _ = self.preflight()
        if not any(a["prefixlen"] == 24 for a in addresses):
            # Do not redirect the entire video subnet: only install a lidar /32 route.
            self.change("address", "add", PREFIX, "dev", INTERFACE, "noprefixroute")
        if not routes:
            self.change("route", "add", LIDAR + "/32", "dev", INTERFACE, "src", HOST)
        addresses, _, _ = self.preflight()
        if not any(a["prefixlen"] == 24 for a in addresses):
            raise NetworkError("eno1 lost the configured address; check NetworkManager/DHCP")
        self.verify_route()
        self.verify_neighbor()
        self.verify_route()
        LOG.info("MID360 network ready; only the authorized address/link changed; "
                 "default routes and persistent profiles unchanged")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true", help="Read-only preflight; no ARP probe or changes")
    mode.add_argument("--apply", action="store_true", help="Configure dedicated link and probe ARP as root")
    parser.add_argument("--takeover-video-address", action="store_true",
                        help="Allow moving only static 192.168.1.50/24 from enp100s0 to eno1")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        network = Network()
        if args.apply:
            network.apply(args.takeover_video_address)
        else:
            network.preflight(args.takeover_video_address)
        return 0
    except (NetworkError, OSError, ValueError, subprocess.SubprocessError) as error:
        LOG.error("MID360 initialization FAILED: %s", error)
        if args.apply:
            LOG.error("Driver must not start. See commands above for temporary address/route changes; "
                      "an authorized video address transfer is not automatically undone.")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
