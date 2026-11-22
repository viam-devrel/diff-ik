# Learned policy in sim: expert-gated DAgger, round 1 — results

Date: 2026-11-22. Spec: `docs/superpowers/specs/2026-11-22-diff-ik-policy-dagger-design.md`; plan:
`docs/superpowers/plans/2026-11-22-diff-ik-policy-dagger.md`.

## Result

Corrections from the policy's own failure states moved SmolVLA from 23/30 to **27/30**. The control (the same
fine-tune without the corrections) scored 22/30. A − B = 5 successes, past the spec's bar of 4, so DAgger earns its
keep. Against the control it fixed 5 seeds and broke none (one-sided sign test p ≈ 0.03). 27/30 is the best gate of
any policy in this project (ACT v2 24, diffusion v3 25).

| model | gate (module/sync, seeds 9100–9129) | t_success median | jerk RMS median | failed seeds |
|---|---|---|---|---|
| SmolVLA v4 (baseline: 20k steps on v3-200) | 23/30 | 9.8 s | 8969 | 9101, 9110, 9111, 9115, 9123, 9124, 9125 |
| B, control: v4 + 10k steps on v3-200 | 22/30 | 9.5 s | 9511 | v4's + 9122 |
| **A, DAgger: v4 + 10k steps on v3-200 + corrections** | **27/30** | 9.5 s | 9001 | 9111, 9123, 9125 |

- DAgger fixed **9101**, the far-corner seed that every earlier policy failed (ACT, diffusion, SmolVLA v2–v4). It
  also fixed **9110**, where the gripper finger covers the block in the wrist camera, and both low-x edge seeds
  (**9115, 9124**) that only SmolVLA had failed.
- Extra steps alone did nothing (22 vs 23). The gain comes from the data, not the training.
- Speed and smoothness are unchanged: the median time to success and jerk match v4's. All 3 failures are timeouts.
- Still failing: 9111 (block on the table's dark stained patch, low contrast), 9123 and 9125.

## Collection

- SmolVLA v4 on seeds 3000–3299 (the training distribution; eval seeds held out), module/sync, with the takeover
  triggers of spec §1 (`clients/python/dagger_run.py`, round `r1`).
- 300 rollouts: the policy succeeded alone in **249 (83%)**; **51 takeovers**, **49 corrections kept** (both failures
  were the expert's own grasp timeouts).
- By trigger: empty_grasp 30 (29 kept), approach_stall 15 (14), drop 6 (6). In 50 of the 51 takeovers the gripper was
  commanded closed: the policy's dominant failure is closing on nothing.
- The sampler never missed a read (at least 74 samples per rollout, about 10 Hz).
- Datasets: `diff-ik-pick-dagger-r1` (`6b035ef8b11eaafd4d756c29`, 49 corrections) and `diff-ik-pick-v3-200-dagger1`
  (`6b035f03db1efb7ab47529d2`, 247 episodes, 22,585 frames). The corrections are 4,430 frames, about 20% of the
  training frames.

## Training

- viam-lerobot-training script v6 (`--pretrained viam:<org>/<model>@<version>`, PR #3). It downloads the registry
  model `diff-ik-pick-smolvla@4` with the job's API key. Two jobs with identical args: 448 px, 10k steps, batch 32,
  last checkpoint, SmolVLA's class-default schedule. LeRobot scaled the schedule to 333 warmup steps and decay to 10k.
  - A `6b036618fedf64c19472fd77` → `diff-ik-pick-smolvla-dagger` v1: final loss 0.012.
  - B `6b036618c438953ace1b8d9a` → `diff-ik-pick-smolvla-ctrl` v1: final loss 0.009.
- Both ran on 2 × A100-SXM4-**80GB**. Every earlier job in this project had 40 GB GPUs, so the GPU type varies between
  jobs.
- The first two jobs (script v5) failed in setup: the package URL on app.viam.com returns 403 without the bearer
  token. Fixed in v6, which sends the client's `Authorization` header.
- Local check before the billed jobs (4 steps from the exported package against `smolvla_base`, on 10 episodes): the
  loss was 4–5× lower from v4, and the normalizer stats came from the new dataset. LeRobot overrides them with the
  dataset's stats on a fine-tune, and keeps the checkpoint's only on resume.

## What the implementation needed (found in review and smoke tests)

- Triggers must look only at a window spent in one phase. Otherwise every normal grasp read as a carry stall, and a
  block resting in opening fingers read as a drop.
- The resource APIs must be imported before connecting: the Python SDK builds clients only for APIs registered at
  connect time.
- Capture's first frame lags `start_capture` by about 0.1 s. The expert holds 0.5 s when the gripper is closed on
  nothing, so the correction opens on that state and records the open as an action. It holds 0.15 s otherwise, and
  checks the block before capture so a stalled policy's correction doesn't open on still frames.

## Next

- **Round 2** on the DAgger model: its 3 failures and new seeds. Expect fewer triggers, since it fails less, so more
  rollouts per correction.
- **The same recipe for ACT or diffusion**, to see whether the corrections transfer or must come from each policy's
  own failures.
- **Fix the camera and table**: widen or move the wrist camera so the fingers don't cover the block, and use a table
  texture without dark patches. 9111 points at the latter.
- Record the per-trigger mix again in round 2: if empty grasps still dominate, a grasp-specific recovery (re-open and
  re-centre) may be cheaper than full takeovers.
