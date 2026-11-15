# Module diff-ik 

Provide a description of the purpose of the module and any relevant information.

## Models

This module provides the following model(s):

- [`devrel:diff-ik:servo`](devrel_diff-ik_servo.md) - Provide a brief description of the model

## Local SO-101 simulation (Phase 3a)

No cloud machine needed:

    (cd ~/src/so-101 && make)            # builds bin/arm
    go build -o bin/diff-ik ./cmd/module
    viam-server -config dev/so101-sim.json   # serves plaintext localhost:8080

Then, from another shell:

    go run ./cmd/cli --address localhost:8080 --insecure --arm arm --task pointing run static
    uv run clients/python/servo_bench.py --address localhost:8080 --insecure --arm arm --task pointing

The paths in `dev/so101-sim.json` are absolute for one laptop; edit `executable_path` for yours.
