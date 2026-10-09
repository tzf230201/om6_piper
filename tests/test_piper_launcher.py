import _bootstrap  # Repository-local imports; no hardware access.
"""Offline launcher checks. subprocess is mocked; never open a robot or camera."""
import contextlib
import io
import unittest
from unittest.mock import patch
import piper


class LauncherTests(unittest.TestCase):
    def test_default_is_doctor_not_gui_or_hardware(self):
        with patch.object(piper,'doctor',return_value=0) as doctor, patch.object(piper.subprocess,'call') as call:
            self.assertEqual(piper.main([]),0);doctor.assert_called_once();call.assert_not_called()

    def test_gui_snapshot_forwarded_unchanged(self):
        args=['gui','--snapshot','../output/example.json','--port','8768']
        with patch.object(piper.subprocess,'call',return_value=0) as call:
            self.assertEqual(piper.main(args),0)
            command=call.call_args.args[0]
            self.assertEqual(command,[piper.sys.executable,str(piper.resource('piper_readonly/web_monitor.py')),*args[1:]])
            self.assertEqual(call.call_args.kwargs['cwd'],piper.PIPER_ROOT)

    def test_maintenance_does_not_add_execution_flags(self):
        for command in ('clear-error','recover','disable','commission'):
            with self.subTest(command=command),patch.object(piper.subprocess,'call',return_value=2) as call:
                self.assertEqual(piper.main([command,'--help']),2)
                self.assertEqual(call.call_args.args[0][2:],['--help'])
                self.assertNotIn('--execute-once',call.call_args.args[0])

    def test_canonical_app_and_shared_dependencies_resolve(self):
        self.assertEqual(piper.resource('trajectory_test.py'),piper.PIPER_ROOT/'trajectory_test.py')
        self.assertEqual(piper.resource('piper_readonly/web_monitor.py'),piper.PIPER_ROOT/'piper_readonly/web_monitor.py')
        self.assertEqual(piper.resource('piper_readonly/commissioning_interlock.json'),piper.WORKSPACE_ROOT/'piper_readonly/commissioning_interlock.json')
        self.assertEqual(piper.resource('tools/video_tools/ffmpeg.exe'),piper.WORKSPACE_ROOT/'tmp/video_tools/ffmpeg.exe')

    def test_unknown_commands_never_launch(self):
        with patch.object(piper.subprocess,'call') as call:
            with self.assertRaises(SystemExit):piper.main(['unrestricted-enable'])
            call.assert_not_called()

    def test_help_does_not_import_or_launch_hardware_scripts(self):
        with contextlib.redirect_stdout(io.StringIO()) as captured,patch.object(piper.subprocess,'call') as call:
            self.assertEqual(piper.main(['--help']),0)
            self.assertIn('scan',captured.getvalue());call.assert_not_called()


if __name__=='__main__':unittest.main()
