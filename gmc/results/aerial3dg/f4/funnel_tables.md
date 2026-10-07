pairs 5000 {'WWEST': 2500, 'GAPW1': 1500, 'S': 1000}
| region | n | distance-passing cand. | endpoint pass | pre-filter pass | A* ROUTE / NRM / NR / UNSURE (pre-filter pass) | pre-filter rejects audited: ROUTE / NRM / NR / UNSURE | CPU s per accepted | CPU h total |
|---|---|---|---|---|---|---|---|---|
| WWEST | 2500 | 26,052 | 12.9% | 74.5% | 2500 / 0 / 0 / 0 | 0 / 37 / 23 / 0 (of 60) | 14.0 | 9.72 |
| GAPW1 | 1500 | 14,709 | 10.6% | 96.3% | 1500 / 3 / 0 / 0 | 0 / 19 / 14 / 0 (of 33) | 8.1 | 3.37 |
| S | 1000 | 874,655 | 5.0% | 2.3% | 1000 / 2 / 12 / 0 | 0 / 16 / 44 / 0 (of 60) | 1.9 | 0.52 |

| region | stage | entered | failed | pass rate | CPU s in stage |
|---|---|---|---|---|---|
| WWEST | endpoint_m001 | 26,052 | 22,695 | 12.9% | 335 |
| WWEST | endpoint_real_dup | 3,357 | 0 | 100.0% |  |
| WWEST | prefilter | 3,357 | 857 | 74.5% | 90 |
| WWEST | astar_real | 2,500 | 0 | 100.0% | 28,896 |
| WWEST | reverify_m001 | 2,500 | 0 | 100.0% | 747 |
| WWEST | evidence (accepted pairs) | 2,500 | | | 5,113 |
| GAPW1 | endpoint_m001 | 14,709 | 13,149 | 10.6% | 95 |
| GAPW1 | endpoint_real_dup | 1,560 | 0 | 100.0% |  |
| GAPW1 | prefilter | 1,560 | 57 | 96.3% | 16 |
| GAPW1 | astar_real | 1,503 | 3 | 99.8% | 10,445 |
| GAPW1 | reverify_m001 | 1,500 | 0 | 100.0% | 188 |
| GAPW1 | evidence (accepted pairs) | 1,500 | | | 1,442 |
| S | endpoint_m001 | 874,655 | 831,136 | 5.0% | 746 |
| S | endpoint_real_dup | 43,519 | 0 | 100.0% |  |
| S | prefilter | 43,519 | 42,505 | 2.3% | 110 |
| S | astar_real | 1,014 | 14 | 98.6% | 431 |
| S | reverify_m001 | 1,000 | 0 | 100.0% | 68 |
| S | evidence (accepted pairs) | 1,000 | | | 511 |
