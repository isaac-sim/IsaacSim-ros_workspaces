## Contribution Guidelines

We welcome contributions from the community! Whether you're fixing a bug, improving documentation,
or proposing a new feature, your help makes this project better for everyone.

### Issue Tracking

- For questions and discussions, join us on [Discord](https://discord.com/channels/827959428476174346/1469128403230789754).
- For bug reports, feature requests, or design discussions, open a [GitHub Issue](https://github.com/isaac-sim/IsaacSim-ros_workspaces/issues).



### Ways to Contribute

- **Bug reports** — Open an issue describing the problem, steps to reproduce, and your environment.
- **Bug fixes** — Submit a pull request with a clear description and a link to the related issue.
- **Enhancements** — Improvements to existing functionality. Opening an issue first is recommended.
- **New features** — New packages, nodes, or APIs. An approved GitHub Issue is **required** before opening a PR.
- **Documentation** — Fixes, clarifications, and additions to `README.md` or package docs.



### Contribution Scope

Pull requests are accepted for changes to:

- ROS 2 workspace source packages (the `src/` directory of each supported distro workspace)
- `README.md`, `CHANGELOG.md`

Where possible, changes should be applied to **all supported ROS 2 distro workspaces** to maintain
feature parity. If a change is distro-specific (e.g., relies on an API only available in one
distro), please explain why in the PR description.

Changes to additional build infrastructure (`build_ros.sh`, `dockerfiles/`) require additional
maintainer involvement — open an issue to discuss before working on these.


### Contribution Types

Different changes carry different levels of risk and review overhead:

| Type | Definition | Prior approval needed |
|---|---|---|
| **Trivial fix** | Docs, comments, typos, formatting — no functional change | No |
| **Bug fix** | Corrects incorrect behavior in existing code | No (issue recommended) |
| **Enhancement** | Improves existing functionality without breaking APIs | Issue recommended |
| **New feature** | Adds new packages, nodes, or changes existing APIs | Issue **required** |

All PRs require review by at least one maintainer.

For new features, please open a GitHub Issue to align on scope before investing time in an
implementation. PRs without prior approval may be closed with a pointer to this process.


### Pull Request Process

1. **Fork** the repository and create a branch from `main`.
2. **Make your changes** within the accepted contribution scope.
3. **Sign off** every commit (see DCO section below):
   ```bash
   git commit -s -m "Your commit message"
   ```
4. **Add license headers** to any new files (Apache-2.0 SPDX header — see below).
5. **Open a pull request** against `main` with a clear title and description.
6. In the PR description, include the change type and affected packages.
7. A maintainer will review your PR and provide feedback.


### Code Style

- **Python:** Follow [PEP 8](https://peps.python.org/pep-0008/) and [ROS 2 Python style guidelines](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
- **C++:** Follow [ROS 2 C++ style guidelines](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
- **License header:** All new source files must include:
  ```python
  # SPDX-License-Identifier: Apache-2.0
  ```
  or the equivalent comment syntax for the file type.


### PR Checklist

Before marking your PR ready for review, confirm:

- [ ] All commits are signed off (`git commit -s`)
- [ ] New files include the Apache-2.0 SPDX license header
- [ ] Code follows the style guidelines above
- [ ] Change is applied to all supported distro workspaces where applicable, or distro-specific reasoning is explained in the PR description
- [ ] Documentation is updated for any new or changed behavior
- [ ] For new features: a linked Issue is referenced in the PR description
- [ ] If your change affects user-facing behavior or adds new functionality, note in the PR description whether the [Isaac Sim documentation](https://docs.isaacsim.omniverse.nvidia.com) needs updating — a maintainer will file the appropriate doc request


### Developer Certificate of Origin

By contributing to this project, you agree to the Developer Certificate of Origin (DCO). All
commits must include a `Signed-off-by` line:

```
Signed-off-by: Your Name <your.email@example.com>
```

This is added automatically with `git commit -s`. The full DCO text is below.

---

```
Developer Certificate of Origin
Version 1.1

Copyright (C) 2004, 2006 The Linux Foundation and its contributors.

Everyone is permitted to copy and distribute verbatim copies of this
license document, but changing it is not allowed.


Developer's Certificate of Origin 1.1

By making a contribution to this project, I certify that:

(a) The contribution was created in whole or in part by me and I
    have the right to submit it under the open source license
    indicated in the file; or

(b) The contribution is based upon previous work that, to the best
    of my knowledge, is covered under an appropriate open source
    license and I have the right under that license to submit that
    work with modifications, whether created in whole or in part
    by me, under the same open source license (unless I am
    permitted to submit under a different license), as indicated
    in the file; or

(c) The contribution was provided directly to me by some other
    person who certified (a), (b) or (c) and I have not modified
    it.

(d) I understand and agree that this project and the contribution
    are public and that a record of the contribution (including all
    personal information I submit with it, including my sign-off) is
    maintained indefinitely and may be redistributed consistent with
    this project or the open source license(s) involved.
```
