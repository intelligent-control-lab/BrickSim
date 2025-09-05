#!/usr/bin/env bash
# Make gdb ignore the venv and any user Python paths
unset PYTHONHOME PYTHONPATH PYTHONUSERBASE PYTHONSTARTUP PYTHONSAFEPATH VIRTUAL_ENV
exec /usr/bin/gdb "$@"
