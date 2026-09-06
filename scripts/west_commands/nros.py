# scripts/west_commands/nros.py
#
# `west nros …` — run the nano-ros CLI with the Zephyr west already knows about.
#
# WHY THIS EXISTS
#
# RFC-0085 D1 puts the user's nano-ros workspace OUTSIDE the west workspace, so
# `nros build` has to be told where Zephyr is. Its ladder is
# `--zephyr-workspace` -> `$ZEPHYR_BASE` -> `$NROS_ZEPHYR_WORKSPACE` -> two path
# conventions, and the first two are the ones a user forgets: a flag has to be
# retyped and an env var is invisible in the command that ran.
#
# Inside a west workspace, none of that guessing is necessary. West already
# knows where Zephyr is — it is a project in the manifest — so this command
# asks west and hands the answer to `nros`. Nothing to remember, nothing to
# retype, and no path in a committed file (RFC-0085 D11 rejects that).
#
# WHAT IT IS DELIBERATELY NOT
#
# A THIN WRAPPER, not a second front end. Every argument is passed through
# untouched and the exit status is propagated, so there is no argument schema
# here to drift out of step with `nros build`'s own. Adding a flag to `nros`
# gives `west nros` that flag for free, on the same day.
#
# THE HONEST LIMIT
#
# West only loads extension commands when it can find a workspace, so this
# command exists when you are standing in the west workspace (or have
# `ZEPHYR_BASE` pointed into one) and not otherwise:
#
#     $ cd /tmp && west nros build …
#     west: unknown command "nros"; do you need to run this inside a workspace?
#
# So it does not remove the two-tree problem, it turns it around: instead of
# naming the Zephyr path (machine-specific, changes per host, easy to forget)
# you name your own workspace with `--workspace` (yours, and you know it). For a
# user who lives in west that is the better half to have to type; for one who
# lives in their nano-ros workspace, `nros build --zephyr-workspace` still is.
#
# Discovery: registered in `scripts/west-commands.yml`, referenced from the
# nano-ros project entry in a workspace `west.yml`. This is now the ONLY
# nano-ros west extension: `west fvp` shared the route until RFC-0064 R5 D4
# retired it (its body was env wiring in front of stock `west build -t run`).
import os
import shutil
import subprocess
import sys

from west.commands import WestCommand


class NrosRun(WestCommand):
    def __init__(self):
        super().__init__(
            'nros',
            'run the nano-ros CLI against this west workspace',
            'Run `nros` with ZEPHYR_BASE set from this west workspace, so a '
            'Zephyr image builds without naming the Zephyr path. Every '
            'argument is forwarded to `nros` unchanged.',
            accepts_unknown_args=True,
        )

    def do_add_parser(self, parser_adder):
        # No arguments of its own — everything belongs to `nros`. Declaring
        # even one would create a name that could collide with a real `nros`
        # flag and would have to be kept in step with it.
        return parser_adder.add_parser(
            self.name,
            help=self.help,
            description=self.description,
            add_help=False,
        )

    def _zephyr_base(self):
        """Where west says Zephyr is.

        Asked of the manifest rather than assumed to be `<topdir>/zephyr`: a
        manifest may place the Zephyr project anywhere, and the point of this
        command is to stop guessing at that path.
        """
        try:
            for project in self.manifest.projects:
                if project.name == 'zephyr':
                    return project.abspath
        except Exception:
            # A workspace whose manifest cannot be read is one west itself will
            # complain about; fall through to the conventional location rather
            # than failing here with a worse message.
            pass
        if self.topdir:
            candidate = os.path.join(self.topdir, 'zephyr')
            if os.path.isdir(candidate):
                return candidate
        return None

    def do_run(self, args, unknown_args):
        nros = shutil.which('nros')
        if nros is None:
            self.die(
                'no `nros` on PATH.\n'
                '  This command runs the nano-ros CLI; it does not bundle one.\n'
                '  Install it, or `source ./activate.sh` in a nano-ros checkout.'
            )

        env = dict(os.environ)
        zephyr_base = self._zephyr_base()
        if zephyr_base:
            # Not overwritten when already set: someone who exported
            # ZEPHYR_BASE has named a Zephyr on purpose, and this command
            # exists to supply a missing answer rather than to overrule a
            # given one.
            env.setdefault('ZEPHYR_BASE', zephyr_base)
            self.dbg(f'west nros: ZEPHYR_BASE={env["ZEPHYR_BASE"]}')
        else:
            self.wrn(
                'west could not locate Zephyr in this workspace, so `nros` '
                'will fall back to its own resolution.'
            )

        # `unknown_args` is everything after `nros`, verbatim.
        proc = subprocess.run([nros, *unknown_args], env=env)
        sys.exit(proc.returncode)
