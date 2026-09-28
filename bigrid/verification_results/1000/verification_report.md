# Bigraph verification analysis report

This report presents the verification results of the bigraph rules acting as a **safety checking mechanism**, for the paper "Bigrid-Bigraph multi-UAV collision-free navigation".

### Query 1

**Meaning**: Q1.1: Safety invariance (does the system ever reach a collision? expected 1)

**Query**: `P=? [G !collision]`

**Verification result**: 1

---

### Query 2

**Meaning**: Q2.1: Probability that a UAS lands successfully (mission completion - any landing)

**Query**: `P=? [F drone_landed]`

**Verification result**: 0.293219097815

---

### Query 3

**Meaning**: Q2.2: Probability that a UAS lands safely (full mission chain under nominal conditions)

**Query**: `P=? [F drone_landed_safe]`

**Verification result**: 0.212849628325

---

### Query 4

**Meaning**: Q2.3: Probability that a low-battery UAS lands (at a charging station or elsewhere)

**Query**: `P=? [F (drone_battery_low_landed | drone_battery_empty_landed)]`

**Verification result**: 0.0640801358012

---

### Query 5

**Meaning**: Q2.4: Probability that a UAS with a communication failure lands

**Query**: `P=? [F drone_comm_los_landed]`

**Verification result**: 0.0734445026502

---

### Query 6

**Meaning**: Q2.5: Probability of emergency landing under combined low battery and communication failure

**Query**: `P=? [F (drone_emergency_landed_1 | drone_emergency_landed_2)]`

**Verification result**: 0.00153481305769

---

### Query 7

**Meaning**: Q2.6: Probability that a low-battery UAS reaches a charging station (emergency path reachability)

**Query**: `P=? [F (drone_lowbatt_at_charger | drone_emptybatt_at_charger)]`

**Verification result**: 0.0912097757246

---

### Query 8

**Meaning**: Q2.7: Probability that a low-battery UAS lands at a charging station and starts charging

**Query**: `P=? [F (drone_charging_1 | drone_charging_2)]`

**Verification result**: 0.0640801358012

---

### Query 9

**Meaning**: Q3.1: Are all landed UAS on the ground layer? (space-state consistency) Identity-bound form: drone_landed_in_air = the same UAS is both Landed and at an air locale (an air locale is uniquely identified by having a DownRoute; ground cells v0..v8 have none). G !"..." must be 1 to actually prove that no landed UAS is left hovering in the air.

**Query**: `P=? [G !drone_landed_in_air]`

**Verification result**: 1

---

