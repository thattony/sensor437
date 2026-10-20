"""
fp_api.py - Locate and import the OpalKelly FrontPanel Python API ("ok" module).

Usage in scripts:   from fp_api import ok

Search order:
  1. $OK_API_PATH            (a directory containing ok.py, or the API root with Python/ok.py)
  2. <package>/opalkelly/FrontPanelAPI   (bundled: macOS universal build of FrontPanel SDK 5.3.6)
  3. ~/vivado-on-silicon-mac/Projects/FrontPanel/API
  4. Windows default install  C:\\Program Files\\Opal Kelly\\FrontPanelUSB\\API\\Python\\<ver>\\x64
  5. plain `import ok` (already installed on sys.path)

On macOS/Linux the native library (libokFrontPanel.dylib/.so) is preloaded with
ctypes RTLD_GLOBAL before importing _ok, which is required because the extension
looks the library up through @rpath.
"""
import ctypes
import glob
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)

_LIBNAMES = {
    "darwin": ["libokFrontPanel.dylib"],
    "linux": ["libokFrontPanel.so"],
    "win32": ["okFrontPanel.dll"],
}


def _candidates():
    env = os.environ.get("OK_API_PATH")
    if env:
        yield env
    yield os.path.join(_PKG, "opalkelly", "FrontPanelAPI")
    yield os.path.expanduser("~/vivado-on-silicon-mac/Projects/FrontPanel/API")
    if sys.platform == "win32":
        base = r"C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python"
        for d in sorted(glob.glob(os.path.join(base, "*", "x64")), reverse=True):
            yield d
    if sys.platform.startswith("linux"):
        yield "/usr/local/lib/opalkelly"


def _try_load(root):
    if not os.path.isdir(root):
        return None
    pydir = os.path.join(root, "Python") if os.path.isfile(os.path.join(root, "Python", "ok.py")) else root
    if not os.path.isfile(os.path.join(pydir, "ok.py")):
        return None
    plat = "win32" if sys.platform == "win32" else ("darwin" if sys.platform == "darwin" else "linux")
    for name in _LIBNAMES[plat]:
        for libdir in (root, pydir):
            lib = os.path.join(libdir, name)
            if os.path.isfile(lib):
                if plat == "win32":
                    os.add_dll_directory(libdir)
                else:
                    ctypes.CDLL(lib, mode=ctypes.RTLD_GLOBAL)
                break
    if pydir not in sys.path:
        sys.path.insert(0, pydir)
    import ok as _ok  # noqa: E402
    return _ok


ok = None
_tried = []
for _root in _candidates():
    _tried.append(_root)
    try:
        ok = _try_load(_root)
    except Exception as exc:  # pragma: no cover - diagnostic path
        _tried[-1] += f"  ({exc})"
        ok = None
    if ok is not None:
        break
if ok is None:
    try:
        import ok  # type: ignore  # noqa: F811
    except ImportError as exc:
        raise ImportError(
            "OpalKelly FrontPanel 'ok' module not found. Install the FrontPanel SDK and set "
            "OK_API_PATH to its API directory. Tried:\n  " + "\n  ".join(_tried)
        ) from exc

__all__ = ["ok"]
