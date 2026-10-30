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
            stack.enter_context(patch.object(transport, "recover_orphan_proxy", return_value=False))
            stack.enter_context(patch.object(transport, "vpn_process_present", return_value=False))
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
        calls = self.check_selection(False, True, [False, True], False)
        self.assertEqual(calls, [call(direct=True), call(direct=False)])

    def test_no_route_stale_listener_is_not_accepted(self):
        self.check_selection(False, True, [False, False], False, error=True)

    def test_working_direct_is_used_even_when_exact_route_hint_is_absent(self):
        self.check_selection(False, True, [True], True)

    def test_orphan_recovery_never_kills_when_vpn_is_active(self):
        with patch.object(transport, "vpn_process_present", return_value=True):
            with patch.object(transport.os, "kill") as kill:
                self.assertFalse(transport.recover_orphan_proxy())
                kill.assert_not_called()

    def test_orphan_recovery_revalidates_identity_before_sigterm(self):
        snapshot = (1234, "Thu Oct 22 11:46:59 2026", "fixed-target")
        with ExitStack() as stack:
            stack.enter_context(patch.object(transport, "vpn_process_present", return_value=False))
            stack.enter_context(patch.object(transport, "orphan_proxy_snapshot", return_value=snapshot))
            stack.enter_context(patch.object(transport, "proxy_listening", return_value=False))
            kill = stack.enter_context(patch.object(transport.os, "kill"))
            self.assertTrue(transport.recover_orphan_proxy())
            kill.assert_called_once_with(1234, transport.signal.SIGTERM)

    def test_changed_process_identity_is_never_killed(self):
        with ExitStack() as stack:
            stack.enter_context(patch.object(transport, "vpn_process_present", return_value=False))
            stack.enter_context(patch.object(transport, "orphan_proxy_snapshot", side_effect=[(1234, "old", "target"), (1234, "new", "target")]))
            kill = stack.enter_context(patch.object(transport.os, "kill"))
            self.assertFalse(transport.recover_orphan_proxy())
            kill.assert_not_called()

    def test_process_inspection_failure_blocks_new_vpn(self):
        with patch.object(transport.subprocess, "run", side_effect=OSError()):
            self.assertTrue(transport.vpn_process_present())

    def test_snapshot_excludes_other_users_and_nonorphan_proxies(self):
        command = f"/opt/homebrew/bin/ocproxy -L {transport.LOCAL_PORT}:{transport.HOST}:{transport.PORT}"
        valid = f"1234 1 {transport.os.getuid()} Thu Oct 22 11:46:59 2026 {command}"
        for line, expected in [
            (valid, 1234),
            (valid.replace("1234 1 ", "1234 42 "), None),
            (valid.replace(f"1 {transport.os.getuid()} ", f"1 {transport.os.getuid() + 1} "), None),
            (valid + " --unrelated", None),
        ]:
            with self.subTest(line=line):
                result = subprocess.CompletedProcess([], 0, stdout=line)
                with patch.object(transport.subprocess, "run", return_value=result):
                    actual = transport.orphan_proxy_snapshot()
                    self.assertEqual(actual[0] if actual else None, expected)

    def test_recovered_orphan_allows_fresh_proxy_start(self):
        script = Path(self.tmp.name) / "vpn-proxy.sh"
        script.write_text("#!/bin/sh\nexit 0\n")
        with ExitStack() as stack:
            stack.enter_context(patch.object(transport, "PROXY_SCRIPT", script))
            stack.enter_context(patch.object(transport, "split_route_present", return_value=False))
            stack.enter_context(patch.object(transport, "ssh_ready", side_effect=[False, False, True]))
            stack.enter_context(patch.object(transport, "proxy_listening", return_value=True))
            stack.enter_context(patch.object(transport, "recover_orphan_proxy", return_value=True))
            stack.enter_context(patch.object(transport, "vpn_process_present", return_value=False))
            stack.enter_context(patch.object(transport, "PROXY_LOG", Path(self.tmp.name) / "proxy.log"))
            new_vpn = stack.enter_context(patch.object(transport.subprocess, "Popen"))
            transport.ensure_proxy()
            new_vpn.assert_called_once()

    def test_active_vpn_without_route_does_not_start_second_vpn(self):
        with ExitStack() as stack:
            stack.enter_context(patch.object(transport, "split_route_present", return_value=False))
            stack.enter_context(patch.object(transport, "ssh_ready", return_value=False))
            stack.enter_context(patch.object(transport, "proxy_listening", return_value=False))
            stack.enter_context(patch.object(transport, "vpn_process_present", return_value=True))
            new_vpn = stack.enter_context(patch.object(transport.subprocess, "Popen"))
            with self.assertRaises(SystemExit):
                transport.ensure_proxy()
            new_vpn.assert_not_called()

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

    def test_prepare_checks_transport_without_reading_a_prompt_or_running_model(self):
        with ExitStack() as stack:
            stack.enter_context(patch.object(transport.sys, "argv", ["codex22.py", "prepare"]))
            ensure = stack.enter_context(patch.object(transport, "ensure_proxy"))
            run_model = stack.enter_context(patch.object(transport, "run_exec"))
            transport.main()
            ensure.assert_called_once_with()
            run_model.assert_not_called()


if __name__ == "__main__":
    unittest.main()
