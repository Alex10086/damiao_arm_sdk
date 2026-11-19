#!/usr/bin/env python3
"""Opt-in CLI safety regressions on a private vcan interface, never real hardware.

DAMIAO_CALIBRATION_BINARY=/path/to/calibrate_joint_limits python3 tests/test_calibrate_limits.py
Set DAMIAO_TEST_DM3507_PMAX only when testing a different linked OpenArm table.
Requires iproute2, the vcan kernel module, and root or passwordless sudo.
"""
import math
import os
import pathlib
import queue
import re
import socket
import struct
import subprocess
import tempfile
import threading
import time
import unittest

FRAME = struct.Struct('=IB3x8s')


class Motors:
    def __init__(self, interface, p3507):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))
        self.sock.settimeout(0.05)
        self.pmax = [12.5] * 3 + [p3507] * 3
        self.vmax = [30.0] * 3 + [50.0] * 3
        self.tmax = [10.0] * 3 + [5.0] * 3
        self.position = [0.0] * 6
        self.velocity = [0.0] * 6
        self.enabled = [False] * 6
        self.missing = set()
        self.bad_master = None
        self.bad_range = None
        self.frames = []
        self.errors = []
        self.running = True
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    @staticmethod
    def quantize(value, maximum, bits):
        return int((value + maximum) * ((1 << bits) - 1) / (2 * maximum))

    def serve(self):
        while self.running:
            try:
                packet = self.sock.recv(FRAME.size)
            except TimeoutError:
                continue
            except OSError as error:
                if self.running:
                    self.errors.append(str(error))
                break
            cid, size, data = FRAME.unpack(packet)
            self.frames.append((cid, size, data))
            if cid != 0x7FF or size != 8 or data[2] not in (0x33, 0xCC) or any(data[4:]):
                self.errors.append('Non-read-only outgoing frame: ' + packet.hex())
                continue
            motor_id = int.from_bytes(data[:2], 'little')
            if not 1 <= motor_id <= 6:
                self.errors.append('Unexpected motor ID: ' + str(motor_id))
                continue
            i = motor_id - 1
            if i in self.missing:
                continue
            if data[2] == 0x33:
                rid = data[3]
                values = {7: 0x11 + i, 8: motor_id, 21: self.pmax[i], 22: self.vmax[i], 23: self.tmax[i]}
                if rid not in values:
                    self.errors.append('Unexpected register read: ' + str(rid))
                    continue
                value = values[rid]
                if i == self.bad_master and rid == 7:
                    value += 1
                if i == self.bad_range and rid == 21:
                    value += 0.5
                payload = data[:4] + struct.pack('<I' if rid in (7, 8) else '<f', value)
            else:
                p = self.quantize(self.position[i], self.pmax[i], 16)
                v = self.quantize(self.velocity[i], self.vmax[i], 12)
                t = self.quantize(0, self.tmax[i], 12)
                payload = bytes([(int(self.enabled[i]) << 4) | motor_id,
                                 p >> 8, p & 255, v >> 4, ((v & 15) << 4) | (t >> 8), t & 255, 35, 30])
            self.sock.send(FRAME.pack(0x11 + i, 8, payload))

    def close(self):
        self.running = False
        self.thread.join(timeout=1)
        self.sock.close()
        if self.thread.is_alive():
            raise RuntimeError('Motor simulator did not terminate')


class CalibrationCLI(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        binary = os.environ.get('DAMIAO_CALIBRATION_BINARY')
        if not binary:
            raise unittest.SkipTest('Opt-in only: set DAMIAO_CALIBRATION_BINARY; never use a hardware interface')
        cls.binary = pathlib.Path(binary).resolve()
        if not cls.binary.is_file():
            raise RuntimeError('Calibration executable not found: ' + str(cls.binary))
        cls.p3507 = float(os.environ.get('DAMIAO_TEST_DM3507_PMAX', '12.5'))
        cls.interface = 'vcal' + str(os.getpid())
        prefix = [] if os.geteuid() == 0 else ['sudo', '-n']
        cls.ip = prefix + ['ip']
        subprocess.run(prefix + ['modprobe', 'vcan'], check=True)
        subprocess.run(cls.ip + ['link', 'add', cls.interface, 'type', 'vcan'], check=True)
        try:
            subprocess.run(cls.ip + ['link', 'set', cls.interface, 'up'], check=True)
        except BaseException:
            subprocess.run(cls.ip + ['link', 'delete', cls.interface], check=True)
            raise

    @classmethod
    def tearDownClass(cls):
        subprocess.run(cls.ip + ['link', 'delete', cls.interface], check=True)

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='damiao-calibration-')
        self.root = pathlib.Path(self.temporary.name)
        self.config = self.root / 'arm.yaml'
        self.output = self.root / 'arm.calibrated.yaml'
        lines = ['# retain this comment', 'can_interface: ' + self.interface, 'enable_fd: false',
                 'joint_count: 6', 'has_gripper: false', 'state_timeout_ms: 500', 'recv_timeout_us: 1000', 'joints:']
        for i in range(6):
            lines += ['  - name: joint' + str(i + 1), '    motor_type: ' + ('DM4310' if i < 3 else 'DM3507'),
                      '    send_can_id: 0x' + format(i + 1, '02x'), '    recv_can_id: 0x' + format(0x11 + i, '02x'),
                      '    control_mode: MIT', '    pos_min: -3.1416 # provisional lower',
                      '    pos_max: 3.1416 # provisional upper', '    vel_max: ' + str(30 if i < 3 else 50),
                      '    tau_max: ' + str(10 if i < 3 else 5), '    kp_max: 500', '    kd_max: 5']
        self.original = '\n'.join(lines) + '\n'
        self.config.write_text(self.original)
        self.motors = Motors(self.interface, self.p3507)

    def tearDown(self):
        self.motors.close()
        self.assertEqual(self.motors.errors, [], 'Calibration must only transmit permitted read queries')
        self.temporary.cleanup()

    def args(self, *extra):
        return [str(self.binary), '--config', str(self.config), '--output', str(self.output),
                '--samples', '3', '--sample-period-ms', '5', *extra]

    def run_check(self):
        return subprocess.run(self.args('--check-only'), text=True, capture_output=True, timeout=20)

    def teach(self, endpoints=None, moving=False, cancel=False, change_source=False):
        if endpoints is None:
            endpoints = [(-0.6 - i * 0.02, 0.8 + i * 0.02) for i in range(6)]
        child = subprocess.Popen(self.args(), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, bufsize=1)
        messages = queue.Queue()
        log = []

        def collect():
            for line in child.stdout:
                messages.put(line)
            messages.put(None)

        reader = threading.Thread(target=collect, daemon=True)
        reader.start()
        deadline = time.monotonic() + 25
        ready_sent = save_sent = False
        try:
            while time.monotonic() < deadline:
                try:
                    line = messages.get(timeout=0.1)
                except queue.Empty:
                    continue
                if line is None:
                    break
                log.append(line)
                # These selectors navigate the actual CLI; assertions concern persisted limits and safety.
                endpoint = re.search(r'joint([1-6]).*端点\s*([AB])', line)
                if 'READY' in line and not ready_sent:
                    child.stdin.write('q\n' if cancel else 'READY\n')
                    child.stdin.flush()
                    ready_sent = True
                elif endpoint and ('Enter' in line or '回车' in line):
                    axis = int(endpoint.group(1)) - 1
                    self.motors.position[axis] = endpoints[axis][0 if endpoint.group(2) == 'A' else 1]
                    if moving:
                        self.motors.velocity[axis] = 0.5
                    child.stdin.write('\n')
                    child.stdin.flush()
                elif 'SAVE' in line and not save_sent:
                    if change_source:
                        self.config.write_text(self.original + '# changed during calibration\n')
                    child.stdin.write('SAVE\n')
                    child.stdin.flush()
                    save_sent = True
            else:
                self.fail('Calibration UI timed out:\n' + ''.join(log))
            return child.wait(timeout=3), ''.join(log)
        finally:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
            child.stdin.close()
            reader.join(timeout=1)
            child.stdout.close()

    def test_healthy_check_has_no_filesystem_or_motor_writes(self):
        result = self.run_check()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue(self.motors.frames)
        self.assertFalse(self.output.exists())
        self.assertEqual(self.config.read_text(), self.original)

    def test_six_axis_limits_include_margin_and_preserve_configuration(self):
        code, log = self.teach()
        self.assertEqual(code, 0, log)
        text = self.output.read_text()
        self.assertEqual(self.config.read_text(), self.original)
        self.assertIn('# retain this comment', text)
        self.assertIn('# provisional lower', text)
        lows = [float(x) for x in re.findall(r'^\s+pos_min:\s*([-+0-9.eE]+)', text, re.M)]
        highs = [float(x) for x in re.findall(r'^\s+pos_max:\s*([-+0-9.eE]+)', text, re.M)]
        for i in range(6):
            self.assertAlmostEqual(lows[i], -0.6 - i * 0.02 + math.radians(3), delta=0.001)
            self.assertAlmostEqual(highs[i], 0.8 + i * 0.02 - math.radians(3), delta=0.001)
        without_limits = lambda s: re.sub(r'(?m)^(\s+pos_(?:min|max):).*$', r'\1', s)
        self.assertEqual(without_limits(text), without_limits(self.original))

    def test_existing_destination_is_not_overwritten(self):
        self.output.write_text('existing calibration\n')
        result = self.run_check()
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.output.read_text(), 'existing calibration\n')

    def test_enabled_motor_is_rejected(self):
        self.motors.enabled[2] = True
        result = self.run_check()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertFalse(self.output.exists())

    def test_missing_motor_is_rejected(self):
        self.motors.missing.add(5)
        result = self.run_check()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertFalse(self.output.exists())

    def test_mismatched_parameter_identity_is_rejected(self):
        self.motors.bad_master = 3
        result = self.run_check()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertFalse(self.output.exists())

    def test_incompatible_physical_scaling_is_rejected(self):
        self.motors.bad_range = 4
        result = self.run_check()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertFalse(self.output.exists())

    def test_moving_joint_does_not_generate_limits(self):
        code, log = self.teach(moving=True)
        self.assertNotEqual(code, 0, log)
        self.assertFalse(self.output.exists())

    def test_identical_endpoints_do_not_generate_limits(self):
        code, log = self.teach([(0.2, 0.2)] * 6)
        self.assertNotEqual(code, 0, log)
        self.assertFalse(self.output.exists())

    def test_encoder_wrap_is_not_mistaken_for_a_large_joint_range(self):
        endpoints = [(-p + 0.1, p - 0.1) for p in self.motors.pmax]
        code, log = self.teach(endpoints)
        self.assertNotEqual(code, 0, log)
        self.assertFalse(self.output.exists())

    def test_changed_source_is_not_silently_used(self):
        code, log = self.teach(change_source=True)
        self.assertNotEqual(code, 0, log)
        self.assertFalse(self.output.exists())
        self.assertEqual(self.config.read_text(), self.original + '# changed during calibration\n')

    def test_operator_cancel_does_not_generate_limits(self):
        self.teach(cancel=True)
        self.assertFalse(self.output.exists())
        self.assertEqual(self.config.read_text(), self.original)


if __name__ == '__main__':
    unittest.main(verbosity=2)
