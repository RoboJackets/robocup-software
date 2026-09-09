# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** |  |
| **Assigned owner(s)** |  |
| **Reviewer(s)** |  |
| **Date started revision** |  |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | _Explain what the package does so a new contributor can understand it._ |
| **Current responsibilities** | _List the responsibilities currently owned by this package._ |
| **Out of scope** | _State what this package must not own or attempt to solve._ |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | _Enter the explicit dependency list from `package.xml`._ |
| **Depended on by** | _List packages, executables, or systems that consume this package._ |


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
