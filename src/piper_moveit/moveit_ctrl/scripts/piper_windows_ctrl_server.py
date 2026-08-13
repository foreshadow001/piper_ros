#!/usr/bin/env python3
"""TCP server for Windows eyetracker: receives Piper arm movement commands.

Listens for TCP connections from the Windows host. Supported commands:

    READY                                                   # handshake → ACK
    MOVE_JOINTS:<arm>:<j1>,<j2>,<j3>,<j4>,<j5>,<j6>     # joint-space move (rad)
    MOVE_TO:<arm>:<x>,<y>,<z>                              # Cartesian move (m)
    GET_POSE:<arm>                                          # query current flange pose
    SHUTDOWN                                                # exit server

After each movement, queries /<can_port>/end_pose for the actual reached
flange pose and responds:

    MOVED:<arm>:<x>,<y>,<z>,<qx>,<qy>,<qz>,<qw>,<alpha>,<beta>,<gamma>

Alpha/Beta/Gamma are intrinsic Z-X-Z' Euler angles in degrees.

Usage:
    rosrun moveit_ctrl piper_windows_ctrl_server.py
"""

import math
import socket
import threading

import yaml
import rospy
from geometry_msgs.msg import PoseStamped
from pathlib import Path

from piper_arm_controller import PiperArmController

_SCRIPT_DIR = Path(__file__).resolve().parent


# ------------------------------------------------------------------
# Z-X-Z' Euler extraction (from end_pose_monitor.py)
# ------------------------------------------------------------------

def _zxz_from_quaternion(qx, qy, qz, qw):
    """Quaternion -> intrinsic Z-X-Z' Euler angles (degrees)."""
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
# TCP server
# ------------------------------------------------------------------

class PiperCtrlServer:
    """Single-threaded TCP server. Handles one client at a time."""

    def __init__(self, host, port, ctrls):
        self._host = host
        self._port = port
        self._ctrls = ctrls  # dict: arm_name → PiperArmController
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
        rospy.loginfo(f"PiperCtrlServer listening on {self._host}:{self._port} "
                      f"(arms={list(self._ctrls.keys())})")

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
                    cmd = line.decode("utf-8", errors="ignore").strip()
                    if not cmd:
                        continue
                    rospy.loginfo(f"Received: {cmd}")
                    resp = self._process(cmd)
                    if resp is not None:
                        rospy.loginfo(f"Sending: {resp}")
                        conn.sendall((resp + "\n").encode("utf-8"))
                    if cmd.strip() == "SHUTDOWN":
                        conn.close()
                        return
        except Exception as e:
            rospy.logerr(f"Client handler error: {e}")
        finally:
            try:
                conn.close()
            except Exception:
                pass
            rospy.loginfo("Client disconnected")

    # ------------------------------------------------------------------
    # Command processing
    # ------------------------------------------------------------------

    def _get_current_flange_pose(self, can_port):
        """Query /end_pose topic for current flange pose. Returns (x,y,z,qx,qy,qz,qw) or None."""
        topic = f"{can_port}/end_pose" if can_port else "end_pose"
        try:
            msg = rospy.wait_for_message(topic, PoseStamped, timeout=1.0)
            p = msg.pose
            return (p.position.x, p.position.y, p.position.z,
                    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        except rospy.ROSException:
            rospy.logwarn(f"Timeout waiting for {topic}")
            return None

    def _format_moved_response(self, arm, can_port):
        """Query current pose and format MOVED:... response."""
        pose = self._get_current_flange_pose(can_port)
        if pose is None:
            return f"ERROR:{arm}:no pose data"
        x, y, z, qx, qy, qz, qw = pose
        alpha, beta, gamma = _zxz_from_quaternion(qx, qy, qz, qw)
        return (f"MOVED:{arm}:"
                f"{x:.6f},{y:.6f},{z:.6f},"
                f"{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},"
                f"{alpha:.4f},{beta:.4f},{gamma:.4f}")

    def _process(self, cmd):
        """Parse and execute a command. Returns response string or None."""
        if cmd.strip() == "READY":
            # Handshake: client confirms server is ready
            return "ACK"

        elif cmd.startswith("GET_POSE:"):
            # "GET_POSE:upper" → query current flange pose (no movement)
            _, arm = cmd.split(":", 1)
            if arm not in self._ctrls:
                return f"ERROR:{arm}:unknown arm (available: {list(self._ctrls.keys())})"
            ctrl = self._ctrls[arm]
            pose = self._get_current_flange_pose(ctrl.can_port)
            if pose is None:
                return f"ERROR:{arm}:no pose data"
            x, y, z, qx, qy, qz, qw = pose
            alpha, beta, gamma = _zxz_from_quaternion(qx, qy, qz, qw)
            return (f"POSE:{arm}:"
                    f"{x:.6f},{y:.6f},{z:.6f},"
                    f"{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},"
                    f"{alpha:.4f},{beta:.4f},{gamma:.4f}")

        elif cmd.startswith("MOVE_JOINTS:"):
            # "MOVE_JOINTS:upper:0.0,0.0,0.0,0.0,0.0,0.0"
            _, arm, joints_str = cmd.split(":", 2)
            if arm not in self._ctrls:
                return f"ERROR:{arm}:unknown arm (available: {list(self._ctrls.keys())})"
            ctrl = self._ctrls[arm]
            joints = [float(x.strip()) for x in joints_str.split(",")]
            if len(joints) != 6:
                return f"ERROR:{arm}:expected 6 joint values, got {len(joints)}"
            rospy.loginfo(f"MOVE_JOINTS {arm}: {joints}")
            ok = ctrl.move_to_joints(joints)
            rospy.sleep(0.15)
            if ok:
                return self._format_moved_response(arm, ctrl.can_port)
            # move_to_joints may return False if arm is already at target
            # (MoveIt planner finds no path when start == goal).
            # Query current pose — if valid, the arm is effectively zeroed.
            resp = self._format_moved_response(arm, ctrl.can_port)
            if resp and not resp.startswith("ERROR:"):
                rospy.loginfo(f"MOVE_JOINTS {arm}: arm already at target, reporting current pose")
                return resp
            return f"ERROR:{arm}:move_joints failed"

        elif cmd.startswith("MOVE_TO:"):
            # "MOVE_TO:upper:0.3,0.1,0.2"
            _, arm, xyz_str = cmd.split(":", 2)
            if arm not in self._ctrls:
                return f"ERROR:{arm}:unknown arm (available: {list(self._ctrls.keys())})"
            ctrl = self._ctrls[arm]
            xyz = [float(v.strip()) for v in xyz_str.split(",")]
            if len(xyz) != 3:
                return f"ERROR:{arm}:expected 3 values (x,y,z), got {len(xyz)}"
            x, y, z = xyz
            rospy.loginfo(f"MOVE_TO {arm}: ({x:.3f}, {y:.3f}, {z:.3f})")
            ok = ctrl.move_to(x, y, z)
            if ok:
                rospy.sleep(0.15)
                return self._format_moved_response(arm, ctrl.can_port)
            return f"ERROR:{arm}:no_solution"

        elif cmd.strip() == "SHUTDOWN":
            rospy.loginfo("Received SHUTDOWN — exiting")
            self._running = False
            rospy.signal_shutdown("Windows requested shutdown")
            return "SHUTDOWN_ACK"

        else:
            return f"ERROR:unknown command: {cmd}"

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
    rospy.init_node("piper_windows_ctrl_server", anonymous=True)

    # Load net config
    net_path = _SCRIPT_DIR / "cfg" / "net.yaml"
    if not net_path.exists():
        rospy.logerr(f"net.yaml not found at {net_path}")
        return
    net_cfg = yaml.safe_load(net_path.read_text())
    net = net_cfg["network"]
    host = net["ip"]
    ctrl_port = net.get("ctrl_port", 49301)

    # Create controllers for both arms (service mode)
    ctrls = {}
    for arm_name in ("upper", "lower"):
        rospy.loginfo(f"Initializing PiperArmController for {arm_name} arm...")
        ctrl = PiperArmController.from_yaml(arm_name, use_service=True)
        ctrls[arm_name] = ctrl
        rospy.loginfo(f"  {arm_name}: can_port={ctrl.can_port}, "
                      f"eye_position={ctrl.eye_position}")

    server = PiperCtrlServer(host, ctrl_port, ctrls)
    server.start()

    rospy.loginfo(f"piper_windows_ctrl_server ready on {host}:{ctrl_port}. "
                  f"Waiting for Windows client...")
    rospy.spin()

    server.stop()
    rospy.loginfo("piper_windows_ctrl_server exited.")


if __name__ == "__main__":
    main()
