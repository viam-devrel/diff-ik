# Learned policy in sim, sub-project 2: ACT and the deployment comparison — Results

Date: 2026-11-21. Spec: `docs/superpowers/specs/2026-11-21-diff-ik-policy-act-design.md`. Plan:
`docs/superpowers/plans/2026-11-21-diff-ik-policy-act.md`.

## v3 dataset

- **Expert changes:** tool tilts of 1–4° at the start, pre-grasp and lift waypoints, drawn from the seed after all
  v1 draws (`bc_pick.sample_scene`), so v2 seeds keep their scenes. Grasp, above-bin and retreat stay vertical. Tilts
  under 1° are sent as tool-down: Viam's orientation vector can't carry a tilt's direction below about 0.81° (the
  server reads it as a tilt toward +x), which made segments through that band jump about 1.5° in one tick.
  `policy_layout.pose_map_from` snaps the same band for every pose the runner sends.
- **Gripper action:** the latched command (`--gripper-key input`, 0/1) in both state and action.
- **10-episode check (seeds 9100–9109):** 10/10 kept. A converted probe showed r02/r12 std 0.020/0.018 (v2:
  0.002/0.001), max tilt about 3.8°, gripper column only 0/1.
- **Capture:** `diff-ik-pick-v3` (`6b01d1fbb11eaafd4d0a331a`), seeds 2000–2099, **100/100 kept**, 9171 frames
  (75–112 per episode, median 90). `diff-ik-pick-v3-eval` (`6b01d1fc9d30069f0fb8f2c1`), seeds 9100–9129,
  **30/30 kept**, 2741 frames. Tag `expert:v2`. Converted with the converter's `--action-space ee --grippers gripper
  --gripper-key input --image-size 224 --fps 10`; no NaNs, `action[t] == state[t+1]`.

## Local ACT (Track 2)

- **Training:** LeRobot `02655292`, MPS, ACT defaults, `chunk_size` 20, `n_action_steps` 20, batch 8, 20k steps
  (about 17 epochs), about 45 min. Loss 23 → 0.51 (5k) → 0.22 (10k) → 0.16 (15k) → 0.13 (20k).
- **Serving:** `vla-policy` (vla 0.3.0) with `-compat` copies that drop `dtype` and `optimizer_fused`
  (the module's LeRobot is older; viam-labs/vla-inference-service#26 aligns them).
- **Open loop (200 v3-eval frames, 20k):** first-row xyz error median 14.6 mm (p90 34.5), last-row 12.9 mm (p90 44.1),
  gripper error median 0.03.
- **Closed loop, module mode, sync, `lead_ms` 100, 10 eval seeds each:**

  | checkpoint | successes |
  |---|---|
  | 5k | 2/10 |
  | 10k | 4/10 |
  | 15k | 1/10 |
  | 20k | 3/10 |

  The 10k checkpoint on all 30 eval seeds: **13/30 (43%)**. Short of the 70% gate.
- **Failure mode:** the policy runs the right sequence (descend, grab, lift, over the bin, open) but lands the grab off
  target: one episode closed with the tool tip 76 mm above and 42 mm beside the block, another at the right height
  but 31 mm off in y. Misses are followed by re-open/re-grab dithering until the 30 s timeout.

### Gripper timing (found and fixed, no effect on success)

The runner fired each gripper command `lead` early, as the spec said. Logging commanded vs measured pose at every
gripper command showed the arm 13–24 mm short of the row pose (mostly in z) at the grab: the lead already makes the
*arm* reach row k at its time, so shifting the gripper too closes it a full lag before the arm arrives. The gripper now
fires at the row's own time (`--grip-early` keeps the old timing). A/B on the 30 eval seeds, 10k checkpoint:

| gripper timing | successes | jerk median | seam median (mm) |
|---|---|---|---|
| row time (fixed) | 13/30 | 10752 | 36.6 |
| lead early (old) | 14/30 | 10698 | 39.0 |

Paired metrics are within noise. The misses come from policy accuracy, not timing.

## Cloud ACT (Track 1)

- **Pipeline:** viam-labs/viam-sequence-to-lerobot#10 and viam-labs/viam-lerobot-training#1 squash-merged (converter
  `3215dc0`). Training script `lerobot-training` v1, then v2 (below), uploaded to DevRelDemos.
- **Containers:** `docker.io/hipsterbrown/viam-lerobot-training:0.1.0-1` (17.3 GB; all curated policies) took about 50 min
  from submit to the program starting, most of it the image pull. `0.1.0-slim-1` (5.0 GB; `lerobot[training,peft]`
  only, ACT/diffusion) started within minutes.
- **Job 1 (`6b01f137…`) failed:** `viam train submit --args` rejects `=` inside a value, so the README's
  `extra="'--policy.chunk_size=50 …'"` can't be submitted; the space form `--policy.chunk_size 50` passed the CLI but the
  script parsed `50` as a flag (`argument --policy.n_action_steps: expected one argument`). Fixed in the script (`extra`
  accepts `--flag value` pairs; branch `extra-space-values`, uploaded as v2; the PR is pending a working `gh`).
- **Job 2 (`6b02006f…`):** slim container, 2 × A100 40GB (`accelerate`, one process per GPU), batch 32 per GPU (64 in
  all), 30k steps (233 epochs of 9171 frames, 10% of episodes held out), lr 3e-5 policy and backbone (√8 × ACT's 1e-5 for
  8× the batch), chunk 20. 0.08 s/step once warm (data 0.004 s: compute-bound, 4 workers per process), 2.45 GB per GPU,
  **43 min of training**. Final train loss 0.028, eval loss 0.155 (best 0.152). Model `diff-ik-pick-act` v1.
- **Open loop (same 200 frames as local):** first-row error median 11.8 mm, but p90 124 mm: on some frames the
  prediction collapses to about the dataset-mean pose, which reads as overfitting (train 0.028 vs eval 0.155).
- **Gate (module, sync, `lead_ms` 100, 30 eval seeds; served from a compat copy on vla 0.3.0):**

  | model | successes | jerk median | seam median / p95 (mm) | row0 median (mm) |
  |---|---|---|---|---|
  | cloud 30k | **20/30 (67%)** | 8632 | 12.0 / 30.7 | 14.7 |
  | local 10k | 13/30 (43%) | 10752 | 36.6 / 80.9 | 21.6 |

  One short of the 21/30 bar.

## Deployment comparison

Cloud ACT model, 30 eval seeds (9100–9129), `lead_ms` 100, async replans every 10 rows (1 s), 30 s timeout. Gripper
at the row's time. Module/sync is the gate run.

| mode / cadence | successes | t_success median | jerk median | seam median / p95 (mm) | row0 median (mm) | arm msgs/s | outcomes |
|---|---|---|---|---|---|---|---|
| module / sync | 20/30 | 7.6 s | 8632 | 12.0 / 30.7 | 14.7 | 0.6 | timeout 10 |
| module / async | 20/30 | 7.9 s | 8787 | 9.7 / 40.6 | 14.5 | 1.0 | timeout 10 |
| client / sync | 21/30 | 7.6 s | 9904 | 12.4 / 28.6 | 14.6 | 29.4 | timeout 9 |
| client / async | 20/30 | 7.6 s | 9572 | 10.9 / 44.5 | 14.4 | 29.4 | timeout 10 |
| move / sync | 7/30 | 28.6 s | 14766 | 12.0 / 47.5 | 14.3 | 2.1 | timeout 20, plan 3 |
| move / async | 4/30 | 28.4 s | 12830 | 62.8 / 195.4 | 14.6 | 2.7 | timeout 22, plan 4 |

Paired per seed (Δ = first − second):

- **module vs client, async:** client jerk higher by a median 654, module smoother on 24/30 seeds; seam and row0 equal
  (Δ 0.4 mm, −0.1 mm); client sends 28× the messages.
- **module vs client, sync:** client jerk higher by a median 612 (module smoother on 18/30); seam and row0 equal.
- **async vs sync, module:** jerk −148 (async lower on 15/30), seam −0.5 mm: no meaningful difference.
- **async vs sync, client:** jerk +2, seam −1.2 mm: no meaningful difference.

### Reading

- **The servo modes tie on success** (20–21/30; ±15 points at n = 30). Success is set by the policy, not the delivery
  path: every servo failure is a timeout from a missed grasp, as in the gate.
- **`servo_trajectory` matches a careful client with one message per chunk** (0.6–1.0/s against 29/s) and is
  smoother: lower jerk on most paired seeds in both cadences, with equal seams and equal row0. This is the result the
  emulated-policy run predicted (`2026-11-policy-streaming.md`), now with a trained policy that closes the loop.
- **Async inference is free with either servo mode:** overlapping chunks didn't cost success or smoothness. The
  module's age alignment and blend absorb the overlap with no client code.
- **Builtin `Move` per row fails the task:** 7/30 sync and 4/30 async, a median 28 s to succeed when it does (against
  7.6 s), and planning errors. Each row is a stop-and-go plan, so the arm falls behind the chunk; the policy then
  observes states it never saw in training. In async, replacing the queue mid-Move makes the seams jump (p95 195 mm).

## Status and follow-ups

- **Gate:** 20/30 against the 21/30 bar; the user chose to proceed (within the ±15-point noise). Levers if a higher
  rate is needed: more episodes (eval loss 0.155 vs train 0.028), or a shorter run.
- **Upstream:** viam-labs/vla-inference-service#26 (LeRobot `02655292`) is open; until it's merged and released, the
  registry vla 0.3.0 needs the compat copy, so `dev/bc-pick-mujoco.json` still points `vla-policy` at a local path.
  The `extra-space-values` fix for viam-labs/viam-lerobot-training is pushed but its PR isn't open yet.
- **diff-ik v0.4.1** (median-lag auto-lead, honest `servo_trajectory` reply) is not released yet.
