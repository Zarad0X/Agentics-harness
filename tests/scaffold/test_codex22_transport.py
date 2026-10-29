"""Regression coverage for VPN routes and stale local forwarding ports."""

import importlib.util
from contextlib import ExitStack
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import call, patch


SPEC = importlib.util.spec_from_file_location(
    "codex22_transport", Path(__file__).resolve().parents[2] / "transport" / "codex22.py"
)
transport = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(transport)


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.lock = patch.object(
            transport, "PROXY_LOCK", Path(self.tmp.name) / "proxy.lock"
        )
        self.lock.start()
        self.addCleanup(self.lock.stop)
        transport._DIRECT_SSH = True

    def check_selection(self, route, listening, probes, expected, error=False):
        with ExitStack() as stack:
            stack.enter_context(patch.object(transport, "split_route_present", return_value=route))
            stack.enter_context(patch.object(transport, "proxy_listening", return_value=listening))
            ready = stack.enter_context(patch.object(transport, "ssh_ready", side_effect=probes))
            new_vpn = stack.enter_context(patch.object(transport.subprocess, "Popen"))
            if error:
                with self.assertRaises(SystemExit):
                    transport.ensure_proxy()
            else:
                transport.ensure_proxy()
            self.assertEqual(transport._DIRECT_SSH, expected)
            new_vpn.assert_not_called()
            return ready.call_args_list

    def test_working_split_vpn_uses_authenticated_direct_transport(self):
        calls = self.check_selection(True, True, [True], True)
        self.assertEqual(calls, [call(direct=True)])

    def test_failed_direct_route_can_reuse_existing_authenticated_forward(self):
        calls = self.check_selection(True, True, [False, True], False)
        self.assertEqual(calls, [call(direct=True), call(direct=False)])

    def test_dead_direct_and_dead_listener_do_not_create_duplicate_vpn(self):
        self.check_selection(True, True, [False, False], False, error=True)

    def test_dead_direct_without_forward_does_not_create_duplicate_vpn(self):
        self.check_selection(True, False, [False], False, error=True)

    def test_no_route_existing_forward_must_authenticate(self):
        calls = self.check_selection(False, True, [True], False)
        self.assertEqual(calls, [call(direct=False)])

    def test_no_route_stale_listener_is_not_accepted(self):
        self.check_selection(False, True, [False], False, error=True)

    def test_probe_timeout_is_a_failed_transport(self):
        with ExitStack() as stack:
            stack.enter_context(patch.object(transport, "ssh_base", return_value=["ssh"]))
            stack.enter_context(patch.object(
                transport.subprocess, "run", side_effect=subprocess.TimeoutExpired("ssh", 15)
            ))
            self.assertFalse(transport.ssh_ready(direct=True))

    def test_proxycommand_uses_selected_endpoint_without_idle_timeout(self):
        for direct, endpoint, port in [
            (True, transport.HOST, transport.PORT),
            (False, "127.0.0.1", str(transport.LOCAL_PORT)),
        ]:
            with self.subTest(direct=direct):
                with ExitStack() as stack:
                    stack.enter_context(
                        patch.object(transport.sys, "argv", ["codex22.py", "connect"])
                    )
                    stack.enter_context(patch.object(transport, "ensure_proxy"))
                    execute = stack.enter_context(patch.object(transport.os, "execv"))
                    transport._DIRECT_SSH = direct
                    transport.main()
                    execute.assert_called_once_with(
                        "/usr/bin/nc", ["nc", endpoint, port]
                    )


if __name__ == "__main__":
    unittest.main()
