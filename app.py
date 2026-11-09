"""Langit Swarm desktop entry point.

    LangitSwarm.exe                 opens the app window (falls back to the web browser)
    LangitSwarm.exe --browser       open in the default web browser instead
    LangitSwarm.exe --safe-mode     app window without GPU acceleration (blank/black window fix)
    LangitSwarm.exe --no-window     server only (e.g. a second operator screen)
    LangitSwarm.exe --port 8765 --host 127.0.0.1

    (from source: python app.py ...)

The console window shows the log; the same text goes to
%USERPROFILE%\\LangitSwarm\\logs\\launcher.log.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import multiprocessing
import os
import socket
import sys
import threading
import time
import traceback
import webbrowser

APP_TITLE = "Langit Swarm - Drone Show Control"
_LOG = None


# ---------------------------------------------------------------------------------- logging
class _Tee:
    """Copy console output into the launcher log file."""

    def __init__(self, stream, log):
        self.stream, self.log = stream, log

    def write(self, s):
        if self.stream is not None:
            try:
                self.stream.write(s)
            except Exception:  # noqa: BLE001 - console gone or cannot encode
                pass
        try:
            self.log.write(s)
            self.log.flush()
        except Exception:  # noqa: BLE001
            pass
        return len(s)

    def flush(self):
        for f in (self.stream, self.log):
            try:
                if f is not None:
                    f.flush()
            except Exception:  # noqa: BLE001
                pass

    def isatty(self):
        return False


def _home() -> str:
    return os.environ.get("LANGIT_HOME") or os.path.join(os.path.expanduser("~"), "LangitSwarm")


def _start_log():
    global _LOG
    try:
        d = os.path.join(_home(), "logs")
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, "launcher.log")
        if os.path.exists(path):
            os.replace(path, os.path.join(d, "launcher.previous.log"))
        _LOG = open(path, "w", encoding="utf-8", buffering=1)
        sys.stdout = _Tee(sys.stdout, _LOG)
        sys.stderr = _Tee(sys.stderr, _LOG)
        return path
    except OSError:
        return None


def say(msg: str):
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


# ------------------------------------------------------------------------------ Windows checks
def _unblock_downloaded_files() -> int:
    """Files unzipped from a download carry Windows' "Mark of the Web"; the .NET runtime then
    refuses to load the app window's libraries (blank window).  Remove it from our own files,
    as PowerShell's Unblock-File does."""
    if os.name != "nt" or not getattr(sys, "frozen", False):
        return 0
    roots = {os.path.dirname(sys.executable), getattr(sys, "_MEIPASS", "")}
    n = 0
    for root in filter(None, roots):
        for dirpath, _, files in os.walk(root):
            for f in files:
                try:
                    os.remove(os.path.join(dirpath, f) + ":Zone.Identifier")
                    n += 1
                except OSError:
                    pass
    return n


def _webview2_version() -> str | None:
    """Installed Microsoft Edge WebView2 Runtime version, or None."""
    if os.name != "nt":
        return None
    try:
        import winreg
    except ImportError:
        return None
    keys = ("{F3017226-FE2A-4295-8BDF-00C3A9A7E4C5}",)       # Evergreen runtime
    best = None
    for hive, sub in ((winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\WOW6432Node\Microsoft\EdgeUpdate\Clients"),
                      (winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\Microsoft\EdgeUpdate\Clients"),
                      (winreg.HKEY_CURRENT_USER, r"SOFTWARE\Microsoft\EdgeUpdate\Clients")):
        for k in keys:
            try:
                with winreg.OpenKey(hive, sub + "\\" + k) as h:
                    v, _ = winreg.QueryValueEx(h, "pv")
                    if v and v != "0.0.0.0":
                        best = best or str(v)
            except OSError:
                pass
    return best


def _dotnet_ok() -> bool:
    if os.name != "nt":
        return False
    try:
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\Microsoft\NET Framework Setup\NDP\v4\Full") as h:
            rel, _ = winreg.QueryValueEx(h, "Release")
            return int(rel) >= 394802                 # .NET Framework 4.6.2
    except OSError:
        return False


# ------------------------------------------------------------------------------------- server
def _free_port(host: str, start: int) -> int:
    for p in range(start, start + 50):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind((host, p))
                return p
            except OSError:
                continue
    raise SystemExit("no free port found")


def _serve(host: str, port: int, ready: threading.Event, failed: list):
    try:
        from aiohttp import web

        from langit.server import build_app

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        runner = web.AppRunner(build_app(), access_log=None)
        loop.run_until_complete(runner.setup())
        loop.run_until_complete(web.TCPSite(runner, host, port).start())
    except BaseException as e:  # noqa: BLE001 - report why the app cannot start
        failed.append(e)
        traceback.print_exc()
        ready.set()
        return
    ready.set()
    try:
        loop.run_forever()
    finally:
        loop.run_until_complete(runner.cleanup())


def _wait():
    say("Langit Swarm keeps running while this window is open. Close this window to quit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


def _open_browser(url: str, why: str = ""):
    if why:
        say(why)
    say(f"Opening Langit Swarm in your web browser: {url}")
    say("If no browser opens, start Microsoft Edge or Google Chrome and go to that address.")
    try:
        webbrowser.open(url)
    except Exception as e:  # noqa: BLE001
        say(f"could not start the browser ({e})")


# ------------------------------------------------------------------------------------- window
def _run_window(url: str, safe_mode: bool, timeout: float) -> bool:
    """Show the app window.  Returns True if the window worked (closed by the user),
    False if the launcher should fall back to the web browser."""
    if os.name == "nt":
        ver = _webview2_version()
        if not ver:
            say("Microsoft Edge WebView2 Runtime was not found, so the app window cannot open.")
            say("Install it from Microsoft (search 'WebView2 Runtime download') to get the app window.")
            return False
        if not _dotnet_ok():
            say(".NET Framework 4.6.2 or newer is missing, so the app window cannot open.")
            return False
        say(f"App window: Microsoft Edge WebView2 {ver}{' (safe mode, no GPU)' if safe_mode else ''}")
    if safe_mode:
        os.environ["WEBVIEW2_ADDITIONAL_BROWSER_ARGUMENTS"] = "--disable-gpu --disable-gpu-compositing"

    import webview  # pywebview: native window using Edge WebView2 on Windows

    from langit.server import CLIENT_EVENTS

    init_failed = threading.Event()

    class _Watch(logging.Handler):
        def emit(self, record):
            msg = record.getMessage()
            if record.levelno >= logging.ERROR:
                say(f"[window] {msg.strip()[:600]}")
            if "initialization failed" in msg.lower():
                init_failed.set()

    logging.getLogger("pywebview").addHandler(_Watch())
    webview.settings["ALLOW_DOWNLOADS"] = True
    window = webview.create_window(APP_TITLE, url + "?shell=window", width=1560, height=940,
                                   min_size=(1200, 720), background_color="#05070d")
    verdict = {"fallback": False}

    def watchdog():
        t0 = time.time()
        while time.time() - t0 < timeout:
            if CLIENT_EVENTS["window_ready"].is_set():
                say("App window ready.")
                return
            if init_failed.is_set():
                break
            time.sleep(0.25)
        verdict["fallback"] = True
        if CLIENT_EVENTS["window_hello"].is_set():
            say("The app window loaded the page but the app did not start in it.")
        else:
            say("The app window stayed blank (its web engine did not load the page).")
        try:
            window.destroy()
        except Exception:  # noqa: BLE001
            pass

    threading.Thread(target=watchdog, daemon=True).start()
    storage = os.path.join(os.environ.get("LOCALAPPDATA") or _home(), "LangitSwarm", "WebView2")
    try:
        os.makedirs(storage, exist_ok=True)
    except OSError:
        storage = None
    kw = dict(gui="edgechromium" if os.name == "nt" else None, private_mode=False, storage_path=storage)
    if os.name != "nt":
        # Linux / macOS window icon.  On Windows pywebview takes the icon from LangitSwarm.exe
        # itself; passing a file there goes through System.Drawing.Icon, which accepts only
        # .ico files and crashes the window thread on anything else.
        base = getattr(sys, "_MEIPASS", None) or os.path.dirname(os.path.abspath(__file__))
        icon = os.path.join(base, "assets", "brand", "icon_256.png")
        if os.path.isfile(icon):
            kw["icon"] = icon
    say("Opening the app window...")
    while True:
        try:
            webview.start(**kw)
            break
        except TypeError as e:          # older pywebview without one of the options
            bad = [k for k in list(kw) if k in str(e)]
            if not bad:
                raise
            for k in bad:
                kw.pop(k)
    # The window is gone.  If the app never started in it (window thread failed, blank
    # window closed) the caller falls back to the browser; otherwise the user closed it.
    return CLIENT_EVENTS["window_ready"].is_set() and not verdict["fallback"]


# --------------------------------------------------------------------------------------- main
def main():
    multiprocessing.freeze_support()
    ap = argparse.ArgumentParser(description="Langit Swarm drone show control")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--browser", action="store_true", help="open in the default web browser")
    ap.add_argument("--safe-mode", action="store_true", help="app window without GPU acceleration")
    ap.add_argument("--no-window", action="store_true", help="run the server only")
    ap.add_argument("--window-timeout", type=float, default=45.0,
                    help="seconds to wait for the app window before falling back to the browser")
    args = ap.parse_args()

    log_path = _start_log()
    try:
        from langit import __version__
    except Exception:  # noqa: BLE001
        __version__ = "?"
    say(f"Langit Swarm {__version__} is starting. This console shows the app's log - keep it open.")
    say("The first start can take up to a minute while Windows checks the program files.")
    if log_path:
        say(f"Log file: {log_path}")
    n = _unblock_downloaded_files()
    if n:
        say(f"Removed the 'downloaded from the internet' block from {n} program files.")

    port = _free_port(args.host, args.port)
    ready = threading.Event()
    failed: list = []
    say("Loading the show engine...")
    th = threading.Thread(target=_serve, args=(args.host, port, ready, failed), daemon=True)
    th.start()
    t0 = time.time()
    while not ready.wait(10):
        say(f"  still loading ({time.time() - t0:.0f} s)...")
        if time.time() - t0 > 600:
            say("The show engine did not start within 10 minutes.")
            _wait()
            return
    if failed:
        say(f"The show engine could not start: {failed[0]!r}")
        say("Please send the log file above to support.")
        _wait()
        return
    url = f"http://{'127.0.0.1' if args.host in ('0.0.0.0', '') else args.host}:{port}/"
    say(f"Langit Swarm running at {url}")

    if args.no_window:
        _wait()
        return
    if args.browser:
        _open_browser(url)
        _wait()
        return
    try:
        if _run_window(url, args.safe_mode, args.window_timeout):
            return                                   # the user closed the app window
        _open_browser(url, "Falling back to the web browser. Tip: try 'LangitSwarm.exe --safe-mode' "
                           "for the app window without GPU acceleration.")
    except Exception as e:  # noqa: BLE001 - no usable window: use the browser
        traceback.print_exc()
        _open_browser(url, f"The app window is unavailable ({e}).")
    _wait()


if __name__ == "__main__":
    main()
