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

## Next

Cloud training, longer and larger-batch, on the same v3 dataset (Track 1 PRs merged: viam-labs/viam-sequence-to-lerobot#10,
viam-labs/viam-lerobot-training#1).
