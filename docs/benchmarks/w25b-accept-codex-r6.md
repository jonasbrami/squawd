1. **Decision: accept v2 conditionally and run W3 run 6 now.** Launch with explicit `VISION_MODEL=coco-nano-seg-v2-640.onnx`; do not promote it as the default yet at [run_single_demo.sh:52](/home/quenouille/drone/scripts/run_single_demo.sh:52). Vehicle recall is 94.3% with zero admitted-class FPs ([w25b_report.json:15](/home/quenouille/drone/evals/out/w25b_eval/w25b_report.json:15), [w25b_report.json:1957](/home/quenouille/drone/evals/out/w25b_eval/w25b_report.json:1957)). That is enough evidence to test the unchanged car-pursuit W3 gate.

2. **Document these R5 failures as scoped trade-offs:**

   - **Persons:** 86.3% overall and four weak cells ([report:21](/home/quenouille/drone/evals/out/w25b_eval/w25b_report.json:21), [report:410](/home/quenouille/drone/evals/out/w25b_eval/w25b_report.json:410)). V2 provides person display and best-effort click only; exclude sustained person pursuit from the demo script and claim.
   - **Far vehicle aspects:** qualify v2 for **10–22 m slant pursuit**, not all-aspect acquisition through 30 m. Both failed cells are 22–30 m ([report:398](/home/quenouille/drone/evals/out/w25b_eval/w25b_report.json:398)); W3 operates near 14–16 m.
   - **Miss streak:** revise the offline limit from 1.0 to **2.0 seconds**. The observed 1.6-second maximum remains well inside the five-second COCO grace. Do not relax W3’s no-LOST requirement.
   - **COCO preservation:** record v2 as **demo-domain, 80-class-compatible**, not generally COCO-preserving. The −9.5-point overall AP50 loss is accepted only because stock v1 remains available.
   - **Latency:** unresolved, not waived. Before default promotion, benchmark v1 and v2 interleaved on a quiet host with Gazebo running; require v2 p50 ≤50 ms, p95 ≤70 ms, and ≤10% regression from v1. The loaded-host results are unusable ([bench640.json:7](/home/quenouille/drone/evals/out/w25b_eval/bench640.json:7)).

3. **Do not remix before run 6.** Partial freezing already underperformed: stage 1 had only 76.9% person recall and 18.2% negative-frame FPs ([w25b_report_s1.json:15](/home/quenouille/drone/evals/out/w25b_eval/w25b_report_s1.json:15), [w25b_report_s1.json:2575](/home/quenouille/drone/evals/out/w25b_eval/w25b_report_s1.json:2575)). A replay-heavier v3 may recover general COCO, but risks sacrificing the domain lift W3 requires.

4. **Reject hybrid for W3.** Two models roughly double inference cost and require class routing, timestamp alignment, duplicate suppression, and contact-fusion changes. That complexity buys person performance outside the gate.

**Bus/truck COCO erosion:** it does not block this demo-scoped model. Bus is absent; TruckDelivery is evaluated directly in Gazebo. It matters only if v2 is advertised or deployed as a general COCO detector—use stock v1 for that role.