# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** |rj_msgs|
| **Assigned owner(s)** |Pooja Dayaratna, Shourik Banerjee|
| **Reviewer(s)** | N/A|
| **Date started revision** |10/11/2026|

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Holds message, action, and service files |
| **Current responsibilities** | Resposible for all published/accessed data between nodes |
| **Out of scope** | Should not attempt to actually publish/read messages |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** |rj_geometry_msgs, std_msgs |
| **Depended on by** |Every other package in the codebase |


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
| **Unit testing** | colcon build |
| **Coverage gaps** | _List untested algorithms, branches, failure paths, and outlier cases._ |
| **Observed outliers** | _Record flaky behavior, nondeterminism, timing spikes, or unexpected results._ |
