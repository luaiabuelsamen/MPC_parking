#!/usr/bin/env python3
"""Local interactive autonomy lab; one persistent native controller per server."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import subprocess
import threading

ROOT = Path(__file__).resolve().parent.parent


def numeric(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("Expected finite numbers")
    return str(value)


def vector(values, length):
    if not isinstance(values, list) or len(values) != length:
        raise ValueError(f"Expected {length} numeric values")
    return " ".join(numeric(x) for x in values)


def car(value):
    return " ".join([vector(value["start"], 3), vector(value["goal"], 3), numeric(value.get("speed", 1.0))])


def command(payload):
    action = payload["action"]
    if action in ("step", "get"):
        return action.upper()
    if action == "load":
        agents, obstacles = payload["agents"], payload.get("obstacles", [])
        if not 1 <= len(agents) <= 8 or len(obstacles) > 40:
            raise ValueError("Use 1–8 cars and up to 40 obstacles")
        return " ".join(["LOAD", str(len(agents)), str(len(obstacles)),
                         *(car(a) for a in agents), *(vector(o, 5) for o in obstacles)])
    if action == "goal":
        return " ".join(["GOAL", numeric(payload["id"]), vector(payload["goal"], 3)])
    if action == "add":
        return "ADD " + car(payload["agent"])
    if action == "obstacles":
        obstacles = payload["obstacles"]
        if len(obstacles) > 40:
            raise ValueError("Use at most forty obstacles")
        return " ".join(["OBSTACLES", str(len(obstacles)), *(vector(o, 5) for o in obstacles)])
    raise ValueError("Unknown action")


class Worker:
    def __init__(self, binary):
        self.process = subprocess.Popen([str(binary)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True, bufsize=1)
        self.lock = threading.Lock()

    def request(self, payload):
        line = command(payload)
        with self.lock:
            if self.process.poll() is not None:
                raise RuntimeError("Native controller exited; restart the lab")
            self.process.stdin.write(line + "\n")
            self.process.stdin.flush()
            result = self.process.stdout.readline()
            if not result:
                raise RuntimeError("Native controller disconnected")
            return json.loads(result)

    def close(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--buck2", default="buck2")
    parser.add_argument("--binary", type=Path)
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    if args.binary:
        binary = args.binary.resolve()
    else:
        build = subprocess.run([args.buck2, "build", "//:live_worker", "--show-output"],
                               cwd=ROOT, text=True, capture_output=True)
        if build.returncode:
            raise SystemExit(build.stderr)
        binary = ROOT / build.stdout.strip().splitlines()[-1].split(maxsplit=1)[1]
    worker = Worker(binary)
    class Handler(BaseHTTPRequestHandler):
        def send(self, status, data, mime="application/json"):
            if isinstance(data, dict):
                data = json.dumps(data, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(data)

        def local_request(self):
            host = self.headers.get("Host", "")
            allowed = {f"127.0.0.1:{args.port}", f"localhost:{args.port}"}
            origin = self.headers.get("Origin")
            return host in allowed and (origin is None or origin in {"http://" + h for h in allowed})

        def do_GET(self):
            if not self.local_request():
                self.send(403, {"ok": False, "error": "Local origin required"}); return
            if self.path == "/":
                self.send(200, (ROOT / "tools" / "live_lab.html").read_bytes(), "text/html; charset=utf-8")
            elif self.path == "/api/state":
                try:
                    self.send(200, worker.request({"action": "get"}))
                except (ValueError, RuntimeError, OSError) as error:
                    self.send(500, {"ok": False, "error": str(error)})
            else:
                self.send(404, {"ok": False, "error": "Not found"})

        def do_POST(self):
            if not self.local_request():
                self.send(403, {"ok": False, "error": "Local origin required"}); return
            if self.path != "/api/command":
                self.send(404, {"ok": False, "error": "Not found"}); return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 65536:
                    raise ValueError("Invalid request size")
                if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                    raise ValueError("Expected application/json")
                payload = json.loads(self.rfile.read(size))
                result = worker.request(payload)
                self.send(200 if result["ok"] else 400, result)
            except (ValueError, KeyError, TypeError) as error:
                self.send(400, {"ok": False, "error": str(error)})
            except (RuntimeError, OSError) as error:
                self.send(500, {"ok": False, "error": str(error)})

        def log_message(self, *_):
            pass

    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    except OSError:
        worker.close()
        raise
    print(f"Autonomy lab: http://127.0.0.1:{args.port} (Ctrl+C to stop)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        worker.close()


if __name__ == "__main__":
    main()
