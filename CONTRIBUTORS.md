# Contributors

Thank you to everyone who builds Langit Swarm.

| Contributor | Role |
|---|---|
| [**@yudhis7**](https://github.com/yudhis7) | Founder and project owner — product vision and requirements, the Langit Swarm brand, testing on Windows |
| Claude (Anthropic) | AI pair programmer — code and documentation, written with @yudhis7 |

## How to contribute

1. Open an [issue](https://github.com/yudhis7/LangitSwarm-/issues) describing the bug or idea.
   For a bug, attach `%USERPROFILE%\LangitSwarm\logs\launcher.log` and a screenshot.
2. Fork the repository, make your change on a branch, and run the tests:
   ```
   python tests/test_engine.py
   ```
3. Open a pull request. Keep changes focused, and describe how you tested them —
   especially for anything that touches flight paths, safety checks or the MAVLink link.

New to the app? Start with the illustrated [User Guide](docs/USER_GUIDE.md).
