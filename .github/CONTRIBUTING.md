# Contributing

Contributions should preserve the project's central rule: range localization must
not make the robot less consistent than the odometry-only baseline.

## Before Opening A Change

1. Search existing issues and pull requests.
2. Keep changes scoped to one behavior or subsystem.
3. For localization tuning, include the relevant exported trace or explain why the
   change is structural rather than data-driven.
4. Do not commit PROS build output, editor caches, terminal logs, or LaTeX sources.

## Development Checks

Run the checks relevant to the change:

```sh
make quick
python3 -m py_compile tools/*.py
python3 tools/localization_tune_analyzer.py src/tune.txt
```

Hardware-dependent changes must state which robot tests were run. If hardware was
not available, say so explicitly and identify the required test route.

## Pull Requests

Pull requests should include:

- a concise explanation of the behavior changed;
- the evidence supporting any tuning-constant change;
- build and analysis results;
- hardware validation status;
- known limitations or follow-up tests.

Avoid unrelated formatting or refactoring in a behavior change. Do not loosen
fusion gates solely to increase correction frequency.
