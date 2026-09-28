# Bigraph verification analysis report

This report presents the verification results of the bigraph rules acting as a **safety checking mechanism**, for the paper "Bigrid-Bigraph multi-UAV collision-free navigation".

### Query 1

**Meaning**: Q1.1: Is a collision state reachable? (expected 0 - proves the rules prevent collisions)

**Query**: `P=? [F collision]`

**Verification result**: 0

---

### Query 2

**Meaning**: Q1.2: Does the system ever reach a collision? (expected 1 - proves the safety invariant)

**Query**: `P=? [G !collision]`

**Verification result**: 1

---

### Query 3

**Meaning**: Q1.3: Can a collision occur within N steps? (used to check rule completeness)

**Query**: `P=? [F<=N collision]`

**Verification result**: 0

---

### Query 4

**Meaning**: Q2.1: Can a UAS land successfully? (mission completion - any landing) Checks whether at least one path under the bigraph rules leads to a successful landing

**Query**: `P>0 [F drone_landed]`

**Verification result**: true

---

### Query 5

**Meaning**: Q2.2: Can a UAS land safely under nominal conditions? (full mission chain - nominal case) Checks whether at least one path under the bigraph rules leads to a safe landing

**Query**: `P>0 [F drone_landed_safe]`

**Verification result**: true

---

### Query 6

**Meaning**: Q2.3: Can a low-battery UAS land? (landing capability under low battery) Covers both landing at a charging station and landing elsewhere Checks whether at least one path under the bigraph rules lets a low-battery UAS land

**Query**: `P>0 [F (drone_battery_low_landed | drone_battery_empty_landed)]`

**Verification result**: true

---

### Query 7

**Meaning**: Q2.4: Can a UAS with a communication failure land? (landing capability under comm failure) Checks whether at least one path under the bigraph rules lets such a UAS land

**Query**: `P>0 [F drone_comm_los_landed]`

**Verification result**: true

---

### Query 8

**Meaning**: Q2.5: Can a UAS with low battery and comm failure perform an emergency landing? (compound fault) Checks whether at least one path under the bigraph rules leads to an emergency landing

**Query**: `P>0 [F (drone_emergency_landed_1 | drone_emergency_landed_2)]`

**Verification result**: true

---

### Query 9

**Meaning**: Q2.6: Can a low-battery UAS reach a charging station? (emergency path reachability) Checks whether a low-battery UAS (flying or landed) can be at a charging station Checks whether at least one path under the bigraph rules reaches a charging station

**Query**: `P>0 [F (drone_at_charger & (drone_battery_low | drone_battery_empty))]`

**Verification result**: true

---

### Query 10

**Meaning**: Q2.7: Can a low-battery UAS land at a charging station and start charging? (charging mechanism) Note: once charging starts, the rules ensure it completes (charging_low/charging_empty rules) Checks whether at least one path under the bigraph rules reaches this state

**Query**: `P>0 [F (drone_charging_1 | drone_charging_2)]`

**Verification result**: true

---

### Query 11

**Meaning**: Q3.1: Are all landed UAS on the ground layer? (space-state consistency)

**Query**: `P=? [G (drone_landed => drone_at_ground)]`

**Verification result**: 1

---

