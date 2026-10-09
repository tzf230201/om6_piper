# Offline tests

Run `piper.cmd test` from the repository root for the explicitly selected
application, GUI and mapping suites. Reports are local and ignored by Git.

Files here are offline unit/integration tests. Original CAN-opening examples are
isolated under `examples/manual/`; they are not part of test collection.
`_bootstrap.py` adds import paths only. No devices are discovered on import.

Some application tests and their recorded fixtures remain shared in the parent
`piper_readonly/` directory. This is intentional: the repository currently relies
on the existing workspace, rather than claiming a standalone fresh-clone setup.
