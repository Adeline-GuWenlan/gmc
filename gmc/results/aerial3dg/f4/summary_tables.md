| robot | n | REACHABLE | failed (rate) | genuine | by-tolerance | export | unverified | classes |
|---|---|---|---|---|---|---|---|---|
| cylinder | 5000 | 3614 | 1386 (27.72%) | 196 | 656 | 534 | 0 | EP-TOL 656, EXPORT-DOMAIN 524, METHOD-TIMEOUT 194, EXPORT-KIN 10, METHOD-ERROR 1, EP-GENUINE 1 |
| sweeper | 5000 | 4819 | 181 (3.62%) | 1 | 176 | 4 | 0 | EP-TOL 176, EXPORT-KIN 4, EP-GENUINE 1 |

| region / robot | n | REACHABLE | genuine | by-tolerance | export | unverified | classes | wall mean / p90 / max (s) |
|---|---|---|---|---|---|---|---|---|
| WWEST/cylinder | 2500 | 2049 | 195 | 206 | 50 | 0 | EP-TOL 206, METHOD-TIMEOUT 194, EXPORT-DOMAIN 41, EXPORT-KIN 9, METHOD-ERROR 1 | 29.3 / 97.7 / 120 |
| GAPW1/cylinder | 1500 | 1191 | 1 | 262 | 46 | 0 | EP-TOL 262, EXPORT-DOMAIN 46, EP-GENUINE 1 | 6.8 / 18.3 / 89 |
| S/cylinder | 1000 | 374 | 0 | 188 | 438 | 0 | EXPORT-DOMAIN 437, EP-TOL 188, EXPORT-KIN 1 | 0.0 / 0.0 / 1 |
| WWEST/sweeper | 2500 | 2440 | 1 | 57 | 2 | 0 | EP-TOL 57, EXPORT-KIN 2, EP-GENUINE 1 | 1.6 / 3.3 / 23 |
| GAPW1/sweeper | 1500 | 1413 | 0 | 87 | 0 | 0 | EP-TOL 87 | 0.8 / 1.7 / 6 |
| S/sweeper | 1000 | 966 | 0 | 32 | 2 | 0 | EP-TOL 32, EXPORT-KIN 2 | 0.0 / 0.0 / 0 |

cylinder: failure rate vs route lateral clearance (ladder rung passed)
| band | n | failed | rate (95 % CI) | genuine | by-tolerance | export | unverified |
|---|---|---|---|---|---|---|---|
| 0 | 92 | 72 | 78.3% (68.8%-85.5%) | 0 | 11 | 61 | 0 |
| 0.5-1.5 | 239 | 191 | 79.9% (74.4%-84.5%) | 10 | 41 | 140 | 0 |
| 2-3 | 340 | 209 | 61.5% (56.2%-66.5%) | 28 | 53 | 128 | 0 |
| 5 | 365 | 153 | 41.9% (37.0%-47.0%) | 41 | 47 | 65 | 0 |
| 10 | 560 | 183 | 32.7% (28.9%-36.7%) | 45 | 60 | 78 | 0 |
| 20 | 1299 | 256 | 19.7% (17.6%-22.0%) | 54 | 146 | 56 | 0 |
| 50 | 1107 | 148 | 13.4% (11.5%-15.5%) | 17 | 130 | 1 | 0 |
| >=100 | 998 | 174 | 17.4% (15.2%-19.9%) | 1 | 168 | 5 | 0 |

cylinder: failure rate vs A* route length / straight distance
| band | n | failed | rate (95 % CI) | genuine | by-tolerance | export | unverified |
|---|---|---|---|---|---|---|---|
| [1, 1.05) | 343 | 37 | 10.8% (7.9%-14.5%) | 1 | 32 | 4 | 0 |
| [1.05, 1.1) | 1927 | 743 | 38.6% (36.4%-40.8%) | 16 | 277 | 450 | 0 |
| [1.1, 1.2) | 802 | 184 | 22.9% (20.2%-26.0%) | 22 | 123 | 39 | 0 |
| [1.2, 1.35) | 963 | 187 | 19.4% (17.0%-22.0%) | 50 | 124 | 13 | 0 |
| [1.35, 1.5) | 573 | 143 | 25.0% (21.6%-28.7%) | 60 | 66 | 17 | 0 |
| [1.5, 2) | 384 | 91 | 23.7% (19.7%-28.2%) | 46 | 34 | 11 | 0 |
| [2, 99) | 8 | 1 | 12.5% (2.2%-47.1%) | 1 | 0 | 0 | 0 |

cylinder: failure rate vs min endpoint clearance ladder of the cylinder (mm)
| band | n | failed | rate (95 % CI) | genuine | by-tolerance | export | unverified |
|---|---|---|---|---|---|---|---|
| <1.5 | 93 | 93 | 100.0% (96.0%-100.0%) | 0 | 93 | 0 | 0 |
| 1.5 | 563 | 563 | 100.0% (99.3%-100.0%) | 0 | 563 | 0 | 0 |
| 2 | 342 | 50 | 14.6% (11.3%-18.8%) | 30 | 0 | 20 | 0 |
| 3-5 | 2923 | 593 | 20.3% (18.9%-21.8%) | 118 | 0 | 475 | 0 |
| >=10 | 1079 | 87 | 8.1% (6.6%-9.8%) | 48 | 0 | 39 | 0 |

sweeper: failure rate vs route lateral clearance (ladder rung passed)
| band | n | failed | rate (95 % CI) | genuine | by-tolerance | export | unverified |
|---|---|---|---|---|---|---|---|
| 0 | 92 | 2 | 2.2% (0.6%-7.6%) | 0 | 2 | 0 | 0 |
| 0.5-1.5 | 239 | 6 | 2.5% (1.2%-5.4%) | 0 | 6 | 0 | 0 |
| 2-3 | 340 | 17 | 5.0% (3.1%-7.9%) | 0 | 15 | 2 | 0 |
| 5 | 365 | 14 | 3.8% (2.3%-6.3%) | 0 | 14 | 0 | 0 |
| 10 | 560 | 10 | 1.8% (1.0%-3.3%) | 0 | 9 | 1 | 0 |
| 20 | 1299 | 49 | 3.8% (2.9%-5.0%) | 1 | 47 | 1 | 0 |
| 50 | 1107 | 34 | 3.1% (2.2%-4.3%) | 0 | 34 | 0 | 0 |
| >=100 | 998 | 49 | 4.9% (3.7%-6.4%) | 0 | 49 | 0 | 0 |

sweeper: failure rate vs A* route length / straight distance
| band | n | failed | rate (95 % CI) | genuine | by-tolerance | export | unverified |
|---|---|---|---|---|---|---|---|
| [1, 1.05) | 343 | 8 | 2.3% (1.2%-4.5%) | 0 | 8 | 0 | 0 |
| [1.05, 1.1) | 1927 | 62 | 3.2% (2.5%-4.1%) | 0 | 59 | 3 | 0 |
| [1.1, 1.2) | 802 | 36 | 4.5% (3.3%-6.2%) | 0 | 36 | 0 | 0 |
| [1.2, 1.35) | 963 | 38 | 3.9% (2.9%-5.4%) | 0 | 37 | 1 | 0 |
| [1.35, 1.5) | 573 | 23 | 4.0% (2.7%-6.0%) | 1 | 22 | 0 | 0 |
| [1.5, 2) | 384 | 14 | 3.6% (2.2%-6.0%) | 0 | 14 | 0 | 0 |
| [2, 99) | 8 | 0 | 0.0% (0.0%-32.4%) | 0 | 0 | 0 | 0 |

sweeper: failure rate vs min endpoint clearance ladder of the cylinder (mm)
| band | n | failed | rate (95 % CI) | genuine | by-tolerance | export | unverified |
|---|---|---|---|---|---|---|---|
| <1.5 | 93 | 26 | 28.0% (19.9%-37.8%) | 0 | 26 | 0 | 0 |
| 1.5 | 563 | 151 | 26.8% (23.3%-30.6%) | 1 | 150 | 0 | 0 |
| 2 | 342 | 1 | 0.3% (0.1%-1.6%) | 0 | 0 | 1 | 0 |
| 3-5 | 2923 | 3 | 0.1% (0.0%-0.3%) | 0 | 0 | 3 | 0 |
| >=10 | 1079 | 0 | 0.0% (0.0%-0.4%) | 0 | 0 | 0 | 0 |

soundness flags: 0
timeout probe: {"n_probed": 50, "requery_timeout_s": 300.0, "requery_outcome": {"REACHABLE": 36, "TIMEOUT": 14}, "finished_wall_s": [174.5, 265.5, 165.4, 204.5, 192.8, 243.0, 191.3, 183.2, 296.7, 143.7, 296.3, 206.8, 150.5, 191.7, 205.5, 224.0, 185.4, 232.8, 250.6, 253.1, 176.8, 242.7, 225.8, 192.1, 129.8, 259.6, 146.3, 211.4, 98.3, 238.8, 185.6, 130.4, 231.2, 181.4, 252.6, 148.5], "verdict_only": {"UNKNOWN": 22, "REACHABLE": 28}, "verdict_only_wall_s": {"min": 0.3833927970845252, "max": 3.647466269088909, "median": 1.1338758519850671}, "stage_share": {"tighten": 0.591, "shortcut": 0.381, "merge_corners": 0.022, "shared_verification": 0.004, "lifting": 0.001}}
buffer0: {'cylinder': {'REACHABLE': 524, 'TIMEOUT': 28, 'UNKNOWN': 105}, 'sweeper': {'REACHABLE': 177}} 
replay probe: {'cylinder': {'map_unknown': 524, 'floors_clear': 10}, 'sweeper': {'floors_clear': 4}} 
reproduced: {'cylinder': {'True': 1241, 'False': 1}, 'sweeper': {'True': 181}} 
probes: {'cylinder': {'kin': 534, 'b0': 657, 'rq': 1242, 'wit': 0, 'epc': 657}, 'sweeper': {'kin': 4, 'b0': 177, 'rq': 181, 'wit': 0, 'epc': 177}}
