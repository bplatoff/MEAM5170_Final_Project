---
name: review-pr
description: Review a teammate's pull request for this repo using the team procedure (D5). Use when asked to review, check or merge a PR.
---
# Review a pull request

1. Fetch without switching your work: `git fetch origin pull/<N>/head:pr-<N>`.
2. Check it merges cleanly: `git merge-tree $(git merge-base master pr-<N>) master pr-<N>` shows no conflict
   markers, or do a trial merge on a throwaway branch.
3. Check out `pr-<N>` (or the trial merge) and run, from the repo root in the project environment:
   - the tests for every folder the PR touches (see `CLAUDE.md`, Commands);
   - the existing tests for the other folders.
4. Check the change against `docs/decisions.md` and the conventions in `CLAUDE.md` (package imports, no generated
   files, no edits inside `Quadruped-PyMPC/`, world frame, units).
5. Check that the PR updates `docs/status.md` if it completes or advances a task.
6. Report: what the PR does, test results with the exact commands, any conflict with a decision, and anything that
   blocks merging. Results count only if they come from committed code.
7. After merge, delete the branch (D6).
