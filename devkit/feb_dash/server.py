#!/usr/bin/env python3
"""FEB Auto dash, host side: runs the relay in the box over `docker exec -i`, keeps the latest state and
serves one page plus a server-sent-events stream on a port every machine on the network (or the
tailnet) can open. Standard library only, so it runs wherever `feb` runs (Linux, macOS, Windows).

    server.py --port 8780 --box feb-box --relay /home/feb/FEB/feb-racing/devkit/feb_dash/relay.py --meta '{...}'
    server.py --cmd "python3 relay.py"       # the relay already on this machine (ROS installed here)

  GET /            the dashboard
  GET /events      SSE: one "state" event with everything, then a "tick" event per relay line
  GET /state.json  the merged state, once
  GET /meta.json   what feb up was started with (driver, mode, track, sim or car)
  GET /tune        the calibration page (colour ranges, camera geometry, camera lock, venue profile)
  GET /cmd?x=frame|scan   ask the relay for one full-resolution camera frame / scan (arrives on the stream)
  POST /camera     {"auto_exposure": bool, "exposure": int, "auto_wb": bool, "white_balance": int} -> the camera node
  POST /save       {"name": "venue", "yaml": "..."} -> <venues dir>/venue.yaml (the driver's config/venues/)
"""
import argparse, json, os, pathlib, queue, signal, subprocess, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = pathlib.Path(__file__).resolve().parent
STATE = {"t": 0.0, "topics": {}, "nodes": [], "relay": "starting"}
LOCK = threading.Lock()
CLIENTS = set()           # queues, one per open SSE connection
META = {}
RELAY_CMD = []
BOX = ""                  # the container the relay runs in ("" when --cmd runs it on this machine)
RELAY_PROC = None         # the running relay, for commands over its stdin
VENUES = None             # where /save writes profiles (pathlib.Path) or None
STOP = threading.Event()


def broadcast(kind, payload):
    data = "event: %s\ndata: %s\n\n" % (kind, json.dumps(payload, separators=(",", ":")))
    for q in list(CLIENTS):
        try:
            q.put_nowait(data)
        except queue.Full:
            pass                   # a slow browser drops ticks, never blocks the others


def relay_loop():
    """Run the relay, merge its lines into STATE, restart it when it ends (box restarted, stack down)."""
    while not STOP.is_set():
        try:
            proc = subprocess.Popen(RELAY_CMD, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        except OSError as e:
            with LOCK:
                STATE["relay"] = "cannot start relay: %s" % e
            broadcast("relay", STATE["relay"])
            STOP.wait(5)
            continue
        global RELAY_PROC
        RELAY_PROC = proc
        with LOCK:
            STATE["relay"] = "connecting"
        broadcast("relay", "connecting")
        threading.Thread(target=lambda: (STOP.wait(), end_relay(proc)), daemon=True).start()   # a stop while blocked in readline
        for line in proc.stdout:
            if STOP.is_set():
                break
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            with LOCK:
                STATE["t"] = msg.get("t", 0.0)
                STATE["relay"] = "live"
                for name, entry in msg.get("topics", {}).items():
                    cur = STATE["topics"].setdefault(name, {})
                    cur["type"], cur["hz"] = entry.get("type"), entry.get("hz", 0.0)
                    if "data" in entry:
                        cur["data"] = entry["data"]
                        cur["at"] = msg.get("t", 0.0)
                if "nodes" in msg:
                    STATE["nodes"] = msg["nodes"]
            broadcast("tick", msg)
        err = proc.stderr.read()[-400:] if proc.stderr else ""
        proc.wait()
        if STOP.is_set():
            break
        with LOCK:
            STATE["relay"] = "relay ended" + (": " + err.strip().splitlines()[-1] if err.strip() else "") + "; retrying"
        broadcast("relay", STATE["relay"])
        STOP.wait(3)
    end_relay(proc)


def end_relay(proc):
    """Close the relay's stdin (it exits on EOF), then make sure: a docker exec client that dies
    leaves its process running inside the box, so ask the box to end it as well."""
    try:
        proc.stdin.close()
    except Exception:
        pass
    if BOX:
        subprocess.run(["docker", "exec", BOX, "pkill", "-f", "feb_dash/relay.py"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
    try:
        proc.wait(3)
    except Exception:
        try:
            proc.terminate()
        except Exception:
            pass


def set_camera(body):
    """Lock or free the RealSense's exposure and white balance through its parameter service, from the box."""
    if not BOX:
        return {"ok": False, "error": "no box to reach the camera from"}
    sets = []
    if "auto_exposure" in body:
        sets.append(("rgb_camera.enable_auto_exposure", "true" if body["auto_exposure"] else "false"))
        if not body["auto_exposure"] and int(body.get("exposure") or 0) > 0:
            sets.append(("rgb_camera.exposure", str(int(body["exposure"]))))
    if "auto_wb" in body:
        sets.append(("rgb_camera.enable_auto_white_balance", "true" if body["auto_wb"] else "false"))
        if not body["auto_wb"] and int(body.get("white_balance") or 0) > 0:
            sets.append(("rgb_camera.white_balance", str(int(body["white_balance"]))))
    out = []
    for name, value in sets:
        cmd = ["docker", "exec", "-u", "feb", BOX, "bash", "-lc", "source /opt/ros/humble/setup.bash; ros2 param set /camera %s %s" % (name, value)]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
            out.append({"param": name, "value": value, "result": (r.stdout + r.stderr).strip()[-120:]})
        except (OSError, subprocess.TimeoutExpired) as e:
            out.append({"param": name, "value": value, "result": str(e)})
    return {"ok": all("Set parameter successful" in o["result"] for o in out), "set": out}


def save_profile(body):
    """Write a venue profile into the driver's config/venues/ (the folder feb passed as --venues)."""
    import re
    name = re.sub(r"[^a-z0-9_-]", "", str(body.get("name", "")).strip().lower().replace(" ", "_"))
    text = str(body.get("yaml", ""))
    if not name:
        return {"ok": False, "error": "give the venue a name (letters, digits, _ -)"}
    if VENUES is None:
        return {"ok": False, "error": "no venues folder: start the dash with feb up --driver ... (or feb dash --driver NAME)", "yaml": text}
    try:
        VENUES.mkdir(parents=True, exist_ok=True)
        path = VENUES / (name + ".yaml")
        path.write_text(text)
    except OSError as e:
        return {"ok": False, "error": str(e)}
    return {"ok": True, "path": str(path), "profile": name}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):        # quiet
        pass

    def send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")      # the results site's Dash page probes /meta.json
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            self.send(200, (HERE / "index.html").read_bytes(), "text/html; charset=utf-8")
        elif path in ("/tune", "/tune.html"):
            self.send(200, (HERE / "tune.html").read_bytes(), "text/html; charset=utf-8")
        elif path == "/cmd":
            q = self.path.split("?", 1)[1] if "?" in self.path else ""
            cmd = dict(kv.split("=", 1) for kv in q.split("&") if "=" in kv).get("x", "")
            ok = False
            if cmd in ("frame", "scan") and RELAY_PROC is not None and RELAY_PROC.stdin:
                try:
                    RELAY_PROC.stdin.write(cmd + "\n"); RELAY_PROC.stdin.flush(); ok = True
                except (OSError, ValueError):
                    pass
            self.send(200 if ok else 503, json.dumps({"ok": ok}).encode(), "application/json")
        elif path == "/state.json":
            with LOCK:
                body = json.dumps({"state": STATE, "meta": META}).encode()
            self.send(200, body, "application/json")
        elif path == "/meta.json":
            self.send(200, json.dumps(META).encode(), "application/json")
        elif path == "/events":
            self.events()
        else:
            self.send(404, b"not found", "text/plain")

    def do_POST(self):
        path = self.path.split("?")[0]
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self.send(400, b'{"error":"bad json"}', "application/json")
        if path == "/camera":
            self.send(200, json.dumps(set_camera(body)).encode(), "application/json")
        elif path == "/save":
            self.send(200, json.dumps(save_profile(body)).encode(), "application/json")
        else:
            self.send(404, b"not found", "text/plain")

    def events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        q = queue.Queue(maxsize=64)
        with LOCK:
            first = "event: state\ndata: %s\n\n" % json.dumps({"state": STATE, "meta": META}, separators=(",", ":"))
        CLIENTS.add(q)
        try:
            self.wfile.write(first.encode())
            self.wfile.flush()
            while not STOP.is_set():
                try:
                    data = q.get(timeout=10)
                except queue.Empty:
                    data = ": keepalive\n\n"
                self.wfile.write(data.encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            CLIENTS.discard(q)


def main():
    global META, RELAY_CMD, BOX, VENUES
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8780)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--box", default="feb-box")
    ap.add_argument("--relay", default="/home/feb/FEB/feb-racing/devkit/feb_dash/relay.py", help="relay.py as the box sees it")
    ap.add_argument("--cmd", help="run this instead of docker exec (ROS on this machine)")
    ap.add_argument("--meta", default="{}", help="JSON shown in the banner")
    ap.add_argument("--watch-stdin", action="store_true", help="stop when stdin closes (feb passes a pipe: when feb dies, so does the dash)")
    ap.add_argument("--venues", help="the driver's config/venues/ folder, where the Tune page saves profiles")
    a = ap.parse_args()
    META = json.loads(a.meta)
    META.setdefault("venues", a.venues)
    VENUES = pathlib.Path(a.venues) if a.venues else None
    if a.cmd:
        RELAY_CMD = a.cmd.split()
    else:
        BOX = a.box
        inner = "source /opt/ros/humble/setup.bash 2>/dev/null; [ -f \"$FEB_WS/install/setup.bash\" ] && source \"$FEB_WS/install/setup.bash\"; exec python3 -u %s" % a.relay
        RELAY_CMD = ["docker", "exec", "-i", "-u", "feb", a.box, "bash", "-lc", inner]
    srv = ThreadingHTTPServer((a.bind, a.port), Handler)
    srv.daemon_threads = True
    threading.Thread(target=relay_loop, daemon=True).start()

    def stop(*_):
        STOP.set()
        threading.Thread(target=srv.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    if a.watch_stdin:
        def watch():
            try:
                while sys.stdin.readline():
                    pass
            except (OSError, ValueError):
                pass
            stop()
        threading.Thread(target=watch, daemon=True).start()
        parent = os.getppid()
        def watch_parent():       # belt and braces: the pipe can stay open in a child that inherited it
            while not STOP.is_set():
                if os.getppid() != parent:
                    stop(); break
                time.sleep(1.0)
        threading.Thread(target=watch_parent, daemon=True).start()
    try:
        srv.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    STOP.set()
    try:
        srv.server_close()
    except Exception:
        pass
    threading.Thread(target=lambda: (time.sleep(5), os._exit(0)), daemon=True).start()   # whatever else hangs, the port is freed
    if RELAY_PROC is not None:
        end_relay(RELAY_PROC)
    os._exit(0)


if __name__ == "__main__":
    main()
