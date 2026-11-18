# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_config_server |
| **Assigned owner(s)** | Pooja Dayaratna |
| **Reviewer(s)** |  |
| **Date started revision** | 11/17/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | This package exists so that rj_ui, rj_vision_filter, rj_vision_receiver, and rj_referee don't have to individually contain the code to subscribe to the field_dimension and game_settings topics. |
| **Current responsibilities** | _List the responsibilities currently owned by this package._ |
| **Out of scope** | _State what this package must not own or attempt to solve._ |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | rj_common, rj_vision_msgs, rj_referee_msgs |
| **Depended on by** | rj_referee, rj_vision_filter, rj_vision_reciever, and rj_ui |


## 3. Operational Expectations

| Field | Details |
| --- | --- |
| **Failure behavior** | _Describe behavior when an upstream input, sensor, network link, radio, or other dependency fails._ |
| **Recovery behavior** | _Describe automatic recovery, retry, fallback, and operator intervention requirements._ |
| **Configuration** | _List required parameters, defaults, validation rules, and configuration files._ |

## 4. Testing

### Test Coverage and Runtime Quality

| Field | Details |
| --- | --- |
| **Unit testing** | _Describe what unit tests were used on this package and how._ |
| **Coverage gaps** | _List untested algorithms, branches, failure paths, and outlier cases._ |
| **Observed outliers** | _Record flaky behavior, nondeterminism, timing spikes, or unexpected results._ |
