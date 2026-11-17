# Model devrel:diff-ik:servo

Provide a description of the model and any relevant information.

## Configuration
The following attribute template can be used to configure this model:

```json
{
"attribute_1": <float>,
"attribute_2": <string>
}
```

### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `attribute_1` | float  | Required  | Description of attribute 1 |
| `attribute_2` | string | Optional  | Description of attribute 2 |

### Example Configuration

```json
{
  "attribute_1": 1.0,
  "attribute_2": "foo"
}
```

## DoCommand

If your model implements DoCommand, provide an example payload of each command that is supported and the arguments that can be used. If your model does not implement DoCommand, remove this section.

### Example DoCommand

```json
{
  "command_name": {
    "arg1": "foo",
    "arg2": 1
  }
}
```


### Status fields

`lag_ms` is the loop's measured estimate of how far the arm trails its commands, capped at `limits.max_lag_ms`
(default 300, at most 1000). If the arm never moves, the command can run up to that much travel ahead before the first resync. `limits.resync_deg` is how far the arm may be from where it should be given that lag (default: 3 ticks
of travel at max joint speed) before the loop resyncs its command to the measured joints. The command-to-measured gap,
and so the size of a resync snap, can reach lag travel + `resync_deg`.
