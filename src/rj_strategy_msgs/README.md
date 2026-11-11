# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_strategy_msgs |
| **Assigned owner(s)** | Pooja Dayaratna, Shourik Banerjee |
| **Reviewer(s)** | Sanat, Nate |
| **Date started revision** | 10/11/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Defines message and service types used for agent-to-agent communication and coordinators in strategy |
| **Current responsibilities** | Responsible for agent state, agent request/response messages, and coordinator messages and services (`AgentRequest`, `AgentResponse`, `AgentResponseVariant`, `AgentState`, `KickerPicker`, `Marking`, `Waller`, `AgentCommunication`). Request and response types live in `msg/request/` and `msg/response/`; these are read and updated by `rj_strategy/include/rj_strategy/agent/communication/communication_gen.py` |
| **Out of scope** | This package should only define message, action, and service types for strategy; the robot intents strategy sends to planning belong in rj_planning_msgs |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | None |
| **Depended on by** | rj_strategy |


## 3. Operational Expectations

| Field | Details |
| --- | --- |
| **Failure behavior** | N/A |
| **Recovery behavior** | N/A |
| **Configuration** | None |

## 4. Testing

### Test Coverage and Runtime Quality

| Field | Details |
| --- | --- |
| **Unit testing** | N/A |
| **Coverage gaps** | N/A |
| **Observed outliers** | N/A |
