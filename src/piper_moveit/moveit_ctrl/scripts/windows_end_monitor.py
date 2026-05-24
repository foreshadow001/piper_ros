#!/usr/bin/env python3
"""TCP server for Windows eyetracker: serves flange poses for both Piper arms.

Listens for TCP connections from the Windows host. On receiving a
``GET_POSE:<upper|lower>`` request, responds with the latest cached flange pose:

    POSE:<arm>:<x>,<y>,<z>,<qx>,<qy>,<qz>,<qw>,<alpha>,<beta>,<gamma>

Alpha/Beta/Gamma are intrinsic Z-X-Z' Euler angles in degrees.

Usage:
    rosrun moveit_ctrl windows_end_monitor.py
"""

import math
import socket
import threading
from pathlib import Path

import yaml
import rospy
from geometry_msgs.msg import PoseStamped

_SCRIPT_DIR = Path(__file__).resolve().parent


# ------------------------------------------------------------------
# Z-X-Z' 提取 (from end_pose_monitor.py)
# ------------------------------------------------------------------

def _zxz_from_quaternion(qx, qy, qz, qw):
    """Quaternion → intrinsic Z-X-Z' Euler angles (degrees)."""
    r02 = 2.0 * (qx * qz + qw * qy)
    r12 = 2.0 * (qy * qz - qw * qx)
    r20 = 2.0 * (qx * qz - qw * qy)
    r21 = 2.0 * (qy * qz + qw * qx)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

    beta = math.acos(max(-1.0, min(1.0, r22)))
    sin_beta = math.sin(beta)

    if abs(sin_beta) > 1e-6:
        alpha = math.atan2(r02, -r12)
        gamma = math.atan2(r20, r21)
    else:
        gamma = 0.0
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qw * qz)
        if beta < math.pi / 2.0:
            alpha = math.atan2(-r01, r00)
        else:
            alpha = math.atan2(r01, r00)

    return math.degrees(alpha), math.degrees(beta), math.degrees(gamma)


# ------------------------------------------------------------------
# Pose cache (per arm)
# ------------------------------------------------------------------

class ArmPoseCache:
    """Subscribes to a namespaced /end_pose topic and caches latest pose."""

    def __init__(self, arm_name, can_port):
        self.arm_name = arm_name
        self._lock = threading.Lock()
        self._pose = None  # geometry_msgs/Pose

        topic = f"{can_port}/end_pose" if can_port else "end_pose"
        self._sub = rospy.Subscriber(topic, PoseStamped, self._callback)
        rospy.loginfo(f"[{arm_name}] Subscribed to /{topic}")

    def _callback(self, msg):
        with self._lock:
            self._pose = msg.pose
            self._cb_count = getattr(self, '_cb_count', 0) + 1
            if self._cb_count <= 3 or self._cb_count % 100 == 0:
                rospy.loginfo(f"[{self.arm_name}] pose #{self._cb_count}: "
                              f"pos=({msg.pose.position.x:.4f},{msg.pose.position.y:.4f},{msg.pose.position.z:.4f}) "
                              f"ori=({msg.pose.orientation.x:.3f},{msg.pose.orientation.y:.3f},{msg.pose.orientation.z:.3f},{msg.pose.orientation.w:.3f})")

    @property
    def pose(self):
        with self._lock:
            return self._pose

    def format_response(self):
        """Return POSE:... string or None if no data yet."""
        p = self.pose
        if p is None:
            return None
        alpha, beta, gamma = _zxz_from_quaternion(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        )
        return (
            f"POSE:{self.arm_name}:"
            f"{p.position.x:.6f},{p.position.y:.6f},{p.position.z:.6f},"
            f"{p.orientation.x:.6f},{p.orientation.y:.6f},{p.orientation.z:.6f},{p.orientation.w:.6f},"
            f"{alpha:.4f},{beta:.4f},{gamma:.4f}"
        )


# ------------------------------------------------------------------
# TCP server
# ------------------------------------------------------------------

class PoseServer:
    """Single-threaded TCP server. Handles one client at a time."""

    def __init__(self, host, port, arm_caches):
        self._host = host
        self._port = port
        self._caches = arm_caches  # dict: arm_name → ArmPoseCache
        self._sock = None
        self._running = False
        self._thread = None

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._host, self._port))
        self._sock.listen(1)
        self._sock.settimeout(1.0)
        self._running = True
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        rospy.loginfo(f"PoseServer listening on {self._host}:{self._port}")

    def _serve(self):
        while self._running and not rospy.is_shutdown():
            try:
                conn, addr = self._sock.accept()
                rospy.loginfo(f"Client connected: {addr}")
                self._handle_client(conn)
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    rospy.logerr(f"Accept error: {e}")

    def _handle_client(self, conn):
        conn.settimeout(5.0)
        buf = b""
        try:
            while self._running and not rospy.is_shutdown():
                try:
                    data = conn.recv(1024)
                except socket.timeout:
                    continue
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    resp = self._process(line.decode("utf-8", errors="ignore").strip())
                    if resp:
                        conn.sendall((resp + "\n").encode("utf-8"))
        except Exception as e:
            rospy.logerr(f"Client handler error: {e}")
        finally:
            conn.close()
            rospy.loginfo("Client disconnected")

    def _process(self, cmd):
        """Parse a request and return a response string (or None)."""
        rospy.loginfo(f"[Server] Received: {repr(cmd)}")
        if not cmd.startswith("GET_POSE:"):
            return f"ERROR:unknown command: {cmd}"
        arm_name = cmd.split(":", 1)[1].strip()
        if arm_name not in self._caches:
            return f"ERROR:unknown arm: {arm_name}"
        cache = self._caches[arm_name]
        pose = cache.pose
        rospy.loginfo(f"[Server] {arm_name} pose cached={pose is not None}")
        if pose is not None:
            rospy.loginfo(f"[Server] {arm_name} sending: "
                          f"pos=({pose.position.x:.4f},{pose.position.y:.4f},{pose.position.z:.4f})")
        resp = cache.format_response()
        if resp is None:
            return f"ERROR:no pose data for {arm_name}"
        rospy.loginfo(f"[Server] Response: {resp[:120]}...")
        return resp

    def stop(self):
        self._running = False
        if self._sock:
            self._sock.close()
        if self._thread:
            self._thread.join(timeout=2.0)


# ------------------------------------------------------------------
# main
# ------------------------------------------------------------------

def main():
    rospy.init_node("windows_end_monitor", anonymous=True)

    # Load net config
    cfg_path = _SCRIPT_DIR / "cfg" / "net.yaml"
    if not cfg_path.exists():
        rospy.logerr(f"net.yaml not found at {cfg_path}")
        return
    cfg = yaml.safe_load(cfg_path.read_text())
    net = cfg["network"]
    host = net["ip"]
    port = net["port"]

    # Load arm configs to get can_port names
    upper_can = "piper_upper"
    lower_can = "piper_lower"
    for arm_name in ("upper", "lower"):
        arm_cfg_path = _SCRIPT_DIR / "cfg" / f"piper_{arm_name}.yaml"
        if arm_cfg_path.exists():
            arm_cfg = yaml.safe_load(arm_cfg_path.read_text())
            can = arm_cfg.get("arm", {}).get("can_port", "")
            if arm_name == "upper":
                upper_can = can
            else:
                lower_can = can

    # Create pose caches for both arms
    caches = {
        "upper": ArmPoseCache("upper", upper_can),
        "lower": ArmPoseCache("lower", lower_can),
    }

    server = PoseServer(host, port, caches)
    server.start()

    rospy.loginfo("windows_end_monitor ready. Waiting for Windows client...")
    rospy.spin()

    server.stop()


if __name__ == "__main__":
    main()
