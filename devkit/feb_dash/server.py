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
    global META, RELAY_CMD, BOX
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8780)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--box", default="feb-box")
    ap.add_argument("--relay", default="/home/feb/FEB/feb-racing/devkit/feb_dash/relay.py", help="relay.py as the box sees it")
    ap.add_argument("--cmd", help="run this instead of docker exec (ROS on this machine)")
    ap.add_argument("--meta", default="{}", help="JSON shown in the banner")
    ap.add_argument("--watch-stdin", action="store_true", help="stop when stdin closes (feb passes a pipe: when feb dies, so does the dash)")
    a = ap.parse_args()
    META = json.loads(a.meta)
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
    try:
        srv.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    STOP.set()


if __name__ == "__main__":
    main()
