"""What a setup step is, and what the machine looks like.

The old design split this knowledge three ways: a `MENU_ITEMS` array in
`setup.sh` held the labels and defaults, the `setup:` recipe in the justfile held
the order, and a `_setup-*` wrapper per option held the condition. Steps that
nobody wrote a wrapper for ran unconditionally and never appeared in the menu --
thirteen of them, including two that write udev rules.

Here a step is one object. If it is not in this list it does not run, and if it
is in this list the UI shows it.
"""

from __future__ import annotations

import hashlib
import os
import platform
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_DIR = REPO_ROOT / "setup"
SCRIPTS_DIR = SETUP_DIR / "scripts"
FILES_DIR = SETUP_DIR / "files"
HARDWARE_DIR = REPO_ROOT / "scripts" / "hardware"

# `all` and `none` are computed rather than declared per step, so a new step
# joins them without being listed anywhere -- see Step.default_for.
PROFILES = ("dev", "vehicle", "all", "none", "ci", "container")
DECLARED = ("dev", "vehicle", "ci", "container")

PROFILE_HELP = {
    "dev": "Laptop, workstation, PC. Dev tools, libraries, kernel socket "
           "buffers and loopback multicast. No sensor or CAN system config.",
    "vehicle": "The cart itself: everything in dev, plus sensor udev rules, "
               "CAN, PTP and camera kernel modules.",
    "all": "Every step, including the slow and opt-in ones.",
    "none": "Nothing preselected. Tick what you want.",
    "ci": "Headless and minimal. Build dependencies only, no prompts.",
    "container": "Building the AutoSDV image. `dev` minus the steps that "
                 "configure a host kernel the container does not own, and "
                 "minus the two that read src/, which the base image has "
                 "none of.",
}

# There was no `orin` profile in the end: a Jetson with no sensors attached is a
# development machine, and one with sensors is the vehicle. Keeping a
# per-board profile meant maintaining a third column that only ever differed
# from `dev` by whether hardware happened to be plugged in.
RETIRED_PROFILES = {"laptop": "dev", "orin": "dev"}

# How a step's recorded state is spelled, everywhere it is shown. The menu also
# offers a tick box, and the two answer different questions -- "run this now?"
# against "does the machine already have it?" -- so neither is a symbol that
# could be mistaken for the other.
STATE_WORDS = {
    "ok": "installed",
    "stale": "script changed",
    "failed": "failed",
    "skipped": "skipped",
    "pending": "",
}

# The heading each choice group is listed under. A group missing from here is
# shown under its bare id: legible, but say what the decision is instead.
CHOICE_LABELS = {
    "tensorrt-engines": "TensorRT engines (pick one, or neither)",
}

# ROS 2 Humble's supported platform. Anything else is somebody's afternoon.
SUPPORTED_OS = ("ubuntu", "22.04")
OS_OK, OS_WARN, OS_ERROR = "ok", "warn", "error"


@dataclass(frozen=True)
class Requires:
    """What a step needs from the machine it runs on.

    `hardware` is a capability name resolved by `Machine.has`. It never blocks a
    step -- someone provisioning a machine before the hardware arrives must be
    able to select anything -- it only annotates the step as not applicable so
    the reason a default is off is visible rather than mysterious.
    """

    sudo: bool = False
    network: bool = True
    arch: tuple[str, ...] = ()          # empty means any
    hardware: str | None = None
    reboot: bool = False


@dataclass
class Step:
    id: str
    label: str
    why: str
    run: list[str]                       # argv, executed without a shell
    requires: Requires = field(default_factory=Requires)
    profiles: dict[str, bool] = field(default_factory=dict)
    group: str = "Core"
    after: tuple[str, ...] = ()          # ordering only, not auto-selection
    note: str = ""                       # printed after a successful run

    # Steps sharing a `choice` name are ALTERNATIVE routes to the same outcome,
    # and the UIs render them as a radio group under one heading rather than as
    # independent tick boxes. Ticking one unticks its siblings, and a profile
    # that would default more than one to on keeps the first.
    #
    # This exists because two equally valid answers to "how should this be
    # done?" cannot be expressed as two checkboxes without also expressing
    # "both" and "neither" -- and "both" was meaningless for the pair that
    # prompted it (download the engines, or build them here: doing both just
    # does the work twice). "Neither" stays meaningful, so a choice group can be
    # left entirely unticked; it is a choice between routes, not a demand to
    # travel.
    choice: str = ""

    # An optional command that asks the MACHINE whether this step's effect is
    # present, rather than asking the state file whether it once ran. Exit 0
    # means present. It exists because a recorded run is not evidence: a reboot
    # drops the loopback MULTICAST flag, a JetPack OTA replaces the OpenCV
    # headers, and an Autoware upgrade leaves the mirrored data tree pointing at
    # a version that is gone. Steps whose effect cannot be undone that way leave
    # this empty and are reported from the state file alone.
    verify: list[str] = field(default_factory=list)

    # Printed, highlighted, AFTER the whole run -- but only when `verify` says
    # the effect is still missing. It exists for the one thing a setup script
    # cannot do for you: the ZED SDK has no apt package, only an interactive
    # installer, so that step checks and advises instead of installing. Advice
    # printed in step order scrolls off the top of a twenty-step run, which is
    # the same as not printing it.
    advice: str = ""

    @property
    def display(self) -> str:
        """The label plus its decision, for lists read outside the menu.

        "Build here, ignoring the published set" is the right label under a
        heading and a riddle in a confirmation list, which has no heading.
        """
        if not self.choice:
            return self.label
        heading = CHOICE_LABELS.get(self.choice, self.choice)
        # The parenthetical belongs to the menu, where it explains the radio
        # buttons; a one-line summary has no radio buttons.
        heading = heading.split(" (")[0]
        return f"{heading}: {self.label}"

    def default_for(self, profile: str) -> bool:
        if profile == "all":
            return True
        if profile == "none":
            return False
        return self.profiles.get(profile, False)

    def digest(self) -> str:
        """Fingerprint of what this step would do.

        Covers the argv and, when the step runs a script from this repo, that
        script's contents. A marker file could only say "ran once"; this is what
        lets the UI say "ran, but the script has changed since" -- the case that
        silently bites when an install script is edited.
        """
        h = hashlib.sha256()
        for part in self.run:
            h.update(part.encode())
            h.update(b"\0")
            candidate = Path(part)
            try:
                in_repo = candidate.is_file() and candidate.is_relative_to(REPO_ROOT)
            except OSError:
                # A sudo step may name a path the user cannot stat, such as
                # /etc/polkit-1/localauthority/...; it is not ours to hash.
                in_repo = False
            if in_repo:
                h.update(candidate.read_bytes())
        return h.hexdigest()[:12]


class Machine:
    """Detected facts about this host, cached for the process lifetime."""

    def __init__(self) -> None:
        self.arch = platform.machine()
        self.is_jetson = Path("/etc/nv_tegra_release").exists()
        self.host_role = self._host_role()
        self.os_release = self._os_release()
        self._caps: dict[str, bool] = {}

    @staticmethod
    def _os_release() -> dict[str, str]:
        values: dict[str, str] = {}
        path = Path("/etc/os-release")
        if not path.is_file():
            return values
        for line in path.read_text().splitlines():
            key, _, value = line.partition("=")
            if key:
                values[key.strip()] = value.strip().strip('"\'')
        return values

    @property
    def os_name(self) -> str:
        rel = self.os_release
        return rel.get("PRETTY_NAME") or " ".join(
            filter(None, (rel.get("NAME", ""), rel.get("VERSION_ID", "")))
        )

    def os_check(self) -> tuple[str, str]:
        """Is this a machine ROS 2 Humble runs on?

        Three outcomes, not two. Humble targets exactly Ubuntu 22.04, and every
        install script here assumes its apt package names -- but a Debian or a
        neighbouring Ubuntu can be made to work, and someone doing that on
        purpose should get a warning rather than a wall. Anything else is not a
        near miss, so it stops by default and `--ignore-os-check` is the way
        past it.
        """
        rel = self.os_release
        distro = rel.get("ID", "").lower()
        version = rel.get("VERSION_ID", "")
        if (distro, version) == SUPPORTED_OS:
            return OS_OK, ""
        family = f"{distro} {rel.get('ID_LIKE', '')}".lower()
        if "ubuntu" in family or "debian" in family:
            return OS_WARN, (
                f"{self.os_name} is not Ubuntu 22.04. ROS 2 Humble is built "
                f"for 22.04; these steps use its apt package names, so some "
                f"will need adjusting."
            )
        return OS_ERROR, (
            f"{self.os_name or 'this system'} is not Ubuntu or Debian. "
            f"ROS 2 Humble is not packaged for it and these steps will not "
            f"work as written."
        )

    @staticmethod
    def _host_role() -> str | None:
        # config/host is gitignored and says which machine this checkout is.
        path = REPO_ROOT / "config" / "host"
        if path.is_file():
            value = path.read_text().strip()
            if value:
                return value
        return None

    def has(self, capability: str) -> bool:
        if capability not in self._caps:
            self._caps[capability] = self._detect(capability)
        return self._caps[capability]

    def _detect(self, capability: str) -> bool:
        if capability == "cuda":
            return self.is_jetson or shutil.which("nvidia-smi") is not None
        if capability == "can":
            return any(Path("/sys/class/net").glob("can*"))
        if capability == "ublox-gnss":
            return self._usb_present("1546")            # u-blox AG
        if capability == "tier4-camera":
            return self._usb_present("2560") or self._usb_present("0525")
        if capability == "ptp-nic":
            return bool(list(Path("/sys/class/ptp").glob("ptp*")))
        if capability == "display":
            return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        return False

    @staticmethod
    def _usb_present(vendor: str) -> bool:
        if shutil.which("lsusb") is None:
            return False
        try:
            out = subprocess.run(
                ["lsusb"], capture_output=True, text=True, timeout=5
            ).stdout
        except (OSError, subprocess.SubprocessError):
            return False
        return f" {vendor}:" in out.replace("ID ", " ")

    def suggested_profile(self) -> str:
        """A starting point, never a verdict. The user can pick any profile.

        Hardware decides, not the board: a Jetson on a desk is a development
        machine and a Jetson in the cart is the vehicle, and the difference
        visible from here is whether the sensors and the bus are attached.
        """
        if self.has("can") or self.has("ublox-gnss") or self.has("tier4-camera"):
            return "vehicle"
        if self.host_role in {"master", "orin"}:
            return "vehicle"
        return "dev"

    def applicable(self, step: Step) -> tuple[bool, str]:
        req = step.requires
        if req.arch and self.arch not in req.arch:
            return False, f"needs {' or '.join(req.arch)}, this host is {self.arch}"
        if req.hardware and not self.has(req.hardware):
            return False, f"{req.hardware} not detected"
        return True, ""
