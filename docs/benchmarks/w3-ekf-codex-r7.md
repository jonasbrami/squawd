1. **Primary: accept a modified combination of process-noise adaptation and bounded corner recapture.** Q inflation alone is insufficient because measurements beyond the 5 m NN gate never reach the EKF ([contacts.py:437](/home/quenouille/drone/agents/vision/contacts.py:437)). The recovery must cover both NN and NIS gates, but only for the designated COCO vehicle.

2. **Turn trigger and process model.** Extend `TrackerConfig` at [contacts.py:42](/home/quenouille/drone/agents/vision/contacts.py:42) with disabled defaults:

   ```text
   maneuver_key=None
   maneuver_gate_m=8
   maneuver_trigger_m=1.0
   maneuver_trigger_hits=2
   maneuver_window_s=2.0
   maneuver_accel_mps2=20
   maneuver_nis_scale=4
   ```

   Configure these only for `maneuver_key="vehicle"` in the COCO profile at [config.py:69](/home/quenouille/drone/agents/vision/config.py:69). Default `TrackerConfig` therefore leaves mover/M3b behavior unchanged.

   Add per-track trigger state at [contacts.py:163](/home/quenouille/drone/agents/vision/contacts.py:163). Before ordinary association, examine the uniquely qualifying same-superclass geometry measurement within 8 m and within 12° of the last accepted image bearing. For predicted velocity `v` and XY innovation `r`, use signed lateral innovation:

   `cross_m = (ve*rn − vn*re) / max(|v|, 1)`

   Arm maneuver mode after two consecutive frames, ≤0.35 seconds apart, with `|cross_m|≥1 m` and the same nonzero sign. This detects the physical sideways departure from CV without confusing ordinary along-track range noise.

   Let `CvEkf.predict()` at [contacts.py:89](/home/quenouille/drone/agents/vision/contacts.py:89) accept an optional Q scale; while armed use `(20/4)²=25`, otherwise exactly `1`. Reset after three normal-gate accepted measurements or the hard two-second timeout.

   Tests in `tests/test_vision_contacts.py`: `test_coco_designated_track_survives_four_right_angle_corners_with_one_id`, `test_maneuver_q_resets_after_three_nominal_hits`, and extend [test_tracker_config_defaults_are_the_contract:82](/home/quenouille/drone/tests/test_vision_contacts.py:82) to prove maneuver recovery is disabled by default.

3. **Corner recapture: accept, but not as an unconditional NIS×4 gate.** At [contacts.py:465](/home/quenouille/drone/agents/vision/contacts.py:465), reserve the unique trigger-qualified measurement for the designated track. During the two-second maneuver window, admit it through `distance≤8 m` and `NIS≤4×9.21`; retain the superclass and 12° bearing-continuity requirements. Future predictions use inflated Q, allowing velocity to rotate rather than repeatedly relying on the relaxed gate.

   If two qualifying vehicle measurements exist, recover neither—the three-car swap risk dominates availability. Tests: `test_corner_recapture_accepts_unique_car_to_truck_measurement` and `test_corner_recapture_refuses_two_plausible_vehicles`. Also assert no candidate/new ID is born from the consumed measurement.

4. **Bearing-fan readoption: reject in `FlightOps`.** `_readopt_contact()` only consumes positioned contacts from `poses()` ([ops.py:141](/home/quenouille/drone/agents/flight/ops.py:141)); a bearing-only orphan supplies no radial coordinate or safe flight target. Inventing a position along a fan could silently switch to another car. The tracker already knows the designated EKF and should preserve it before removal. If the primary still misses, the fallback belongs inside `VisionContacts`: during the armed window only, fuse a **unique** same-key bearing-only detection within 8° of the last accepted bearing into the existing designated EKF—never mint/adopt a new position. Pin rejection with `test_readoption_refuses_unpositioned_vehicle_candidate`.

5. **Reject IMM/coordinated-turn or heading-aware state for now.** An IMM is mathematically cleaner but materially expands model selection, covariance tuning, and tests. Detector boxes provide no reliable vehicle heading. The bounded high-Q CV mode directly models the instantaneous waypoint corner and automatically returns to the proven straight-line filter.

6. **Do not weaken or renumber W3.** Re-lock-per-corner would gut sustained pursuit: the observed next window can be 43–50 seconds away ([w3-run6.md:25](/home/quenouille/drone/docs/benchmarks/w3-run6.md:25)). Keep the existing ≤8-second re-lock allowance for unrelated dropout, but add a mandatory corner sub-gate: **one contiguous engagement must traverse a complete ~50-second lap, at least four 90° corners, with the same contact ID, no LOST/manual re-lock, and MEASURED recovery within two seconds of each corner.**