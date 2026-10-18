# SPDX-License-Identifier: Apache-2.0
"""Open Horology Studio from Python.

Every method runs the same engine as the ``ohs`` command line and returns its ``--json`` output
as a dictionary, so anything the CLI can do is available here with the same names and units
(millimetres, degrees, seconds per day). A method returns a :class:`Result`, a ``dict`` with the
exit code of the command attached: ``ok`` is false when a check failed (exit code 1, for example a
tolerance run above its limit or a DFM error). Bad input raises :class:`OhsError` (exit code 2).

API version 1.0 is frozen: names, parameters and defaults listed in ``api_1_0.json`` change only
in a new major version. New keyword arguments with defaults may be added in minor versions.
"""
from __future__ import annotations

import json as _json
import os as _os
import tempfile as _tempfile
from typing import Any, Dict, Iterable, List, Optional, Sequence, Union

from . import _native

API_VERSION = "1.0"
__version__ = _native.version()

__all__ = [
    "API_VERSION", "OhsError", "Result", "Project", "run",
    "gear", "materials", "thermal", "verify_physics", "lessons", "lesson", "check_lesson", "calibre_template",
]


class OhsError(Exception):
    """The command rejected its input or failed (exit code 2)."""

    def __init__(self, message: str, exit_code: int = 2, stderr: str = ""):
        super().__init__(message)
        self.exit_code = exit_code
        self.stderr = stderr


class Result(dict):
    """The parsed JSON of a command, with ``exit_code``, ``stdout`` and ``stderr`` attached."""

    exit_code: int = 0
    stdout: str = ""
    stderr: str = ""

    @property
    def ok(self) -> bool:
        return self.exit_code == 0


def run(*args: Any, json: bool = True, check: bool = True) -> Union[Result, str]:
    """Run ``ohs <args>`` in this process.

    With ``json=True`` (the default) ``--json`` is added and the output parsed. ``check=True``
    raises :class:`OhsError` for exit code 2; exit code 1 (a failed check) never raises.
    """
    argv = [str(a) for a in args]
    if json:
        argv = ["--json"] + argv
    code, out, err = _native.run(argv)
    if code == 2 and check:
        raise OhsError(err.strip() or f"ohs {' '.join(argv)} failed", code, err)
    if not json:
        return out
    text = out.strip()
    try:
        data = _json.loads(text) if text else {}
    except ValueError as exc:
        raise OhsError(f"could not parse the output of ohs {' '.join(argv)}: {exc}", code, err) from exc
    result = Result(data) if isinstance(data, dict) else Result({"items": data})
    result.exit_code, result.stdout, result.stderr = code, out, err
    return result


def _opt(flag: str, value: Any) -> List[str]:
    return [] if value is None else [flag, str(value)]


def _toml_study(name: str, changes: Iterable[Dict[str, Any]], verify: bool) -> str:
    lines = ["[study]", f'name = "{name}"', f"verify = {'true' if verify else 'false'}"]
    for change in changes:
        keys = [k for k in ("scale", "set", "add") if k in change]
        if "parameter" not in change or len(keys) != 1:
            raise OhsError("each change needs 'parameter' and exactly one of scale, set, add")
        lines += ["", "[[change]]", f'parameter = "{change["parameter"]}"', f"{keys[0]} = {float(change[keys[0]])!r}"]
    return "\n".join(lines) + "\n"


class Project:
    """A rigged movement: the ``.hsproj`` file that ``ohs import`` writes."""

    def __init__(self, path: Union[str, "_os.PathLike[str]"]):
        self.path = _os.fspath(path)
        if not _os.path.exists(self.path):
            raise OhsError(f"no project file {self.path}")

    @classmethod
    def import_step(cls, step: str, output: str, beat_rate: Optional[float] = None, preset: Optional[str] = None,
                    up: str = "z") -> "Project":
        """Import a STEP assembly and auto-rig it."""
        args: List[Any] = ["import", step, "-o", output, "--up", up] + _opt("--beat-rate", beat_rate) + _opt("--preset", preset)
        run(*args)
        return cls(output)

    def rig_check(self, strict: bool = False) -> Result:
        return run("rig", "check", self.path, *(["--strict"] if strict else []))

    def kinematics(self, beat_rate: Optional[float] = None) -> Result:
        return run("kinematics", self.path, *_opt("--beat-rate", beat_rate))

    def explain(self, part: Optional[str] = None, train: bool = False) -> Result:
        return run("explain", self.path, *([part] if part else []), *(["--train"] if train else []))

    def torque(self, wind: float = 1.0, calibre: Optional[str] = None) -> Result:
        return run("torque", self.path, "--wind", wind, *_opt("--calibre", calibre))

    def timekeeping(self, calibre: Optional[str] = None) -> Result:
        return run("timekeeping", self.path, *_opt("--calibre", calibre))

    def simulate(self, duration: str = "60s", level: str = "reduced", position: str = "dial_up", wind: float = 1.0,
                 calibre: Optional[str] = None) -> Result:
        return run("simulate", self.path, "--duration", duration, "--level", level, "--position", position, "--wind", wind,
                   *_opt("--calibre", calibre))

    def timegrapher(self, duration: str = "30s", position: str = "dial_up", wind: float = 1.0, jitter_us: float = 0.0,
                    seed: int = 1, calibre: Optional[str] = None) -> Result:
        return run("timegrapher", self.path, "--duration", duration, "--position", position, "--wind", wind,
                   "--jitter-us", jitter_us, "--seed", seed, *_opt("--calibre", calibre))

    def whatif(self, changes: Optional[Sequence[Dict[str, Any]]] = None, study: Optional[str] = None, name: str = "what-if",
               verify: bool = False, calibre: Optional[str] = None) -> Result:
        """Compare the baseline with a modified design. Give ``changes`` (dicts with ``parameter`` and one of
        ``scale``, ``set``, ``add``) or the path of a study TOML file."""
        if (changes is None) == (study is None):
            raise OhsError("give either changes or study")
        if study is not None:
            return run("whatif", self.path, study, *(["--verify"] if verify else []), *_opt("--calibre", calibre))
        with _tempfile.TemporaryDirectory() as tmp:
            path = _os.path.join(tmp, "study.toml")
            with open(path, "w", encoding="utf-8") as f:
                f.write(_toml_study(name, changes or [], False))
            return run("whatif", self.path, path, *(["--verify"] if verify else []), *_opt("--calibre", calibre))

    def tolerance(self, samples: int = 10000, seed: int = 42, table: Optional[str] = None, fail_above: float = 0.02) -> Result:
        return run("tolerance", self.path, "--samples", samples, "--seed", seed, "--fail-above", fail_above, *_opt("--tolerances", table))

    def dfm(self, process: str = "mill,wheel-cutting", rules: Optional[str] = None, plugins: Sequence[str] = (),
            step: Optional[str] = None) -> Result:
        args: List[Any] = ["dfm", self.path, "--process", process] + _opt("--rules", rules) + _opt("--step", step)
        for p in plugins:
            args += ["--plugin", p]
        return run(*args)

    def collisions(self, step: Optional[str] = None) -> Result:
        return run("collisions", self.path, *_opt("--step", step))

    def fea_tooth(self, part: str, load_n: Optional[float] = None, material: Optional[str] = None, wind: float = 1.0) -> Result:
        return run("fea", self.path, "--part", part, "--case", "tooth-bending", "--wind", wind, *_opt("--load-n", load_n), *_opt("--material", material))

    def fea_shock(self, part: str, g: float = 5000.0, size: float = 0.2, modes: int = 0, material: Optional[str] = None,
                  step: Optional[str] = None) -> Result:
        return run("fea", self.path, "--part", part, "--case", "shock", "--g", g, "--size", size, *_opt("--modes", modes or None),
                   *_opt("--material", material), *_opt("--step", step))

    def draw(self, part: str, output: str, step: Optional[str] = None) -> Result:
        return run("draw", self.path, "--part", part, "-o", output, *_opt("--step", step))

    def report(self, prefix: str, formats: Sequence[str] = ("html", "pdf"), structural: bool = True, timekeeping: bool = True,
               samples: int = 10000, step: Optional[str] = None, calibre: Optional[str] = None) -> Result:
        args: List[Any] = ["report", self.path, "-o", prefix, "--format", ",".join(formats), "--samples", samples] + _opt("--step", step) + _opt("--calibre", calibre)
        if not structural:
            args.append("--no-fea")
        if not timekeeping:
            args.append("--no-timekeeping")
        return run(*args)


def gear(teeth: int, module: float, pressure_angle: float = 20.0, bore: float = 0.0, thickness: float = 0.5, name: str = "gear",
         material: str = "brass_leaded", step: Optional[str] = None, dxf: Optional[str] = None, svg: Optional[str] = None,
         pdf: Optional[str] = None, profile_only: bool = False) -> Result:
    """Generate an involute wheel and write the files you ask for."""
    args: List[Any] = ["gear", "--teeth", teeth, "--module", module, "--pressure-angle", pressure_angle, "--bore", bore, "--thickness", thickness,
                       "--name", name, "--material", material]
    args += _opt("--step-out", step) + _opt("--dxf", dxf) + _opt("--svg", svg) + _opt("--pdf", pdf)
    if profile_only:
        args.append("--profile-only")
    return run(*args)


def materials() -> Result:
    """The material catalogue."""
    return run("materials")


def thermal(hairspring: str = "nife_hairspring", balance: str = "brass_leaded") -> Result:
    """Rate change per kelvin of a hairspring and balance pair."""
    return run("thermal", "--hairspring", hairspring, "--balance", balance)


def verify_physics(full: bool = False) -> Result:
    """Re-run the solvers against analytical answers."""
    return run("verify", "physics", *(["--full"] if full else []))


def lessons() -> Result:
    return run("lesson", "list")


def lesson(lesson_id: str, project: Optional[str] = None) -> Result:
    """A guided lesson with the numbers of ``project`` (default: the built-in demo movement)."""
    return run("lesson", "show", lesson_id, *_opt("--project", project))


def check_lesson(lesson_id: str, project: Optional[str] = None, answer: Optional[float] = None, study: Optional[str] = None,
                 cutout: Optional[str] = None, solution: bool = False) -> Result:
    args: List[Any] = ["lesson", "check", lesson_id] + _opt("--project", project) + _opt("--answer", answer) + _opt("--study", study) + _opt("--cutout", cutout)
    if solution:
        args.append("--solution")
    return run(*args)


def calibre_template(beat_rate: float = 25200) -> str:
    """The default calibre parameters as TOML text, to edit."""
    return str(run("calibre", "init", "--beat-rate", beat_rate, json=False))
