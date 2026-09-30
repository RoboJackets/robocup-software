# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | `rj_protos` |
| **Assigned owner(s)** | Sanat Dhanyamraju |
| **Reviewer(s)** | Cameron Lyon, Nate Wert |
| **Date started revision** | 9/1/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Contains protobuf files for  SSL-owned interfaces such as game controller and vision processor. |
| **Current responsibilities** | Owns the protocol for recieving data from SSL-owned interfaces. |
| **Out of scope** | Does not contain any logic behind how these messages are parsed or how the data is used. |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | None |
| **Depended on by** | `rj_common`, `rj_radio`, `rj_referee`, `rj_utils`, `rj_vision_receiver` |


## 3. Operational Expectations

| Field | Details |
| --- | --- |
| **Failure behavior** | If the SSL protos migrate to a different repo, this will no longer stay up-to-date with SSL. |
| **Recovery behavior** | Modify the package to submodule the new repo. |
| **Configuration** | CMakeLists.txt contains the list of protos we build, this needs to be updated when we want to use a new proto from SSL. |

## 4. Testing

### Test Coverage and Runtime Quality

| Field | Details |
| --- | --- |
| **Unit testing** | No testing exists as this package contains no code. |
| **Coverage gaps** | N/A |
| **Observed outliers** | N/A |
