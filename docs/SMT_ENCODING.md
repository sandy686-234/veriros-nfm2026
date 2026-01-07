# SMT Encoding Details

VeriROS-Exec encodes the heterogeneous multi-robot task allocation problem as:

**Decision Variables:**
- assign_t_r: Task t assigned to robot r (binary)
- s_t: Start time of task t (real)
- makespan: Total completion time (real)

**Constraints (QF_LIRA Logic):**
- C1: Each task assigned to exactly one robot
- C2: Capability matching
- C3: Non-overlapping tasks on same robot
- C4: Deadline satisfaction
- C5: Resource mutual exclusion
- C6: Makespan definition

**Objective:** Minimize makespan

See paper for detailed formulation.
