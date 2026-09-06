# Adversarial Research Audit — Certified SE(2) Gaussian Mobility Compiler + Robot-Conditioned Mobility Coreset

**Date:** 2026-09-05
**Auditor:** cloud agent, no access to NYU HPC, `/scratch/wg2381/splathjb`, SLURM, the dataset, or the repo.
**Every number attributed to the project below is quoted or arithmetically derived from the brief.** Derived numbers are marked **[derived]** and the arithmetic is shown. Nothing was run.

**Literature verification level.** `arxiv.org` and several mirrors (`ar5iv`, `alphaxiv`, project pages on `github.io`) are blocked by this environment's egress proxy. Citations below were established via web search result metadata plus fetches of the hosts that *were* reachable (GitHub, Springer, ACM, Wikipedia-class sources). Consequently:

- **[V]** = title/authors/venue/ID confirmed from at least two independent search results or a fetched page.
- **[S]** = known from search snippets only; ID and title are reliable, **detailed claims about method internals should be re-read from the PDF before you cite them in a paper.**

Papers dated after 2025-05 are past my training cutoff and are **[S]** by construction. I flag those explicitly. **Do not paste any [S] detail into a submission without opening the PDF.**

---

## Executive summary — the five things that matter

1. **Two internal inconsistencies in the reported results must be resolved before anything else.** (a) The headline gate error is **+2e-5**, but the sign convention implied by your own geometry-only control (−0.346 = conservative, −0.510 = sealed) makes **positive = optimistic**, which contradicts "~40 arms, every gate error ≤ 0 (never optimistic)." Either the two statements use different conventions, or they refer to different sides of the sandwich (M_possible vs M_safe) and the report conflates them, or the headline result is 2e-5 unsound. (b) The coreset's certified clearance lower bound (0.021715945) is **larger** than the uncompressed one (0.02171590806) by 3.7e-8 **[derived]**. A coreset that only ever *grows* obstacles cannot certify *more* clearance than the map it was derived from, if both bounds come from the same estimator on the same path. Both must be explained in writing before the numbers go anywhere.

2. **The headline 17.3× is not the robot-conditioning result.** Your own surface-only control says a free geometric filter gets 484→124 at zero gate error. The residual attributable to robot conditioning is therefore at most **124/28 = 4.43×** **[derived]** — and even that is unverified, because *the coreset was never run on the surface-only variant*. That single missing cell is the cheapest, most decisive experiment you have.

3. **"Coresets are ordered by robot size" is a one-line theorem, not an empirical finding — and the direction you observed is the one monotonicity predicts.** Sealing is monotone in erosion radius: if a merge disconnects `erode(free, r₀)`, it disconnects `erode(free, r)` for all `r > r₀`. Therefore validating at the *largest* probe radius certifies all smaller radii, and a coreset built for the large robot is adequate for the small one, never the converse. Your 2×2 adequacy matrix confirms exactly this. Prove it, don't measure it — and then harvest the payoff: **one coreset built at r_max serves the entire downward-closed robot family**, which is what makes the coarsening cost amortizable.

4. **The certified sandwich, as an idea, does not clearly beat conservative octree inflation with unknown-space tracking.** OctoMap already emits occupied/free/unknown, which is a sandwich. Your real delta is that the (x,y) side is discretization-free and the θ side is a certified *continuous* slab rather than a stack of sampled yaw layers — and SE(2) NavMesh (arXiv 2607.01454, Jul 2026) does the sampled-yaw-layer version, published two months ago. That delta belongs to the compiler, which is the HRM-colliding part.

5. **Verdict: not a T-RO paper as it stands; a credible RA-L / ICRA paper if you land the real-data importer and beat HRM's released C++ code on its own benchmarks.** The compiler is subsumed by Ruan et al. 2023. The coreset layer is a genuine but narrow contribution currently resting on one synthetic scene family, one robot for the headline, an artifact-inflated compression number, and a one-sided guarantee for a two-sided claim.

---

# 1. LITERATURE

## 1(a) 3DGS / point-cloud compression and pruning *for planning or safety*, not rendering

The honest finding of this section: **almost nothing exists in the intersection.** There is a large and fast-moving 3DGS-for-planning literature and a large 3DGS-compression literature, and they do not talk to each other. The compression literature optimizes PSNR/SSIM/LPIPS at fixed MB; the planning literature consumes whatever splats it is given. That gap is real and it is the strongest thing this project has going for it. It is also, for the same reason, a gap other groups will notice in the next twelve months.

**Splat-Nav: Safe Real-Time Robot Navigation in Gaussian Splatting Maps** — Timothy Chen, Ola Shorinwa, Joseph Bruno, Javier Yu, Weijia Zeng, Keiko Nagami, Philip Dames, Mac Schwager. arXiv:2403.02751; IEEE Transactions on Robotics 2025, DOI 10.1109/TRO.2025.3552348. https://arxiv.org/abs/2403.02751 **[V]**
*What it does:* two modules — Splat-Plan builds a safe-by-construction polytope corridor through the GSplat scene using ellipsoid-level-set collision constraints, then fits a Bézier curve inside the corridor; Splat-Loc does recursive monocular pose estimation against the splat point cloud. Replans >2 Hz, localizes ~25 Hz. Evaluated on Stonehenge, Statues, Flight(room), and Old Union/Adirondacks, each in a dense and a sparse variant (reported counts: Stonehenge 116K/12K, Statues 201K/18K, Flight 281K/4K **[S]** — read Table III before citing), 100 start/goal pairs per scene, against a point-cloud planner, RRT\* and NeRF-Nav.
*How it differs:* Splat-Nav consumes **all** primitives at query time and reasons in R³ position space with a sphere/ellipsoid robot; there is no scene reduction step, no orientation dependence, and no *certificate about the map itself* — its guarantee is "this corridor is free w.r.t. these ellipsoids," not "these ellipsoids are a sufficient summary of the scene." Note the dense/sparse variants: they already ran a two-point compression ablation, but the sparse variants come from **training with fewer Gaussians**, not from a principled reduction, and the comparison is on planning success, not on a preserved topological invariant. This is the closest published thing to your ablation and it is the one a reviewer will cite at you.

**SAFER-Splat: A Control Barrier Function for Safe Navigation with Online Gaussian Splatting Maps** — Timothy Chen, Aiden Swann, Javier Yu, Ola Shorinwa, Riku Murai, Monroe Kennedy III, Mac Schwager. arXiv:2409.09868; IEEE ICRA/conference version 2025 (IEEE Xplore doc 11128723). https://arxiv.org/abs/2409.09868 **[V]**
*What it does:* a CBF-based minimally-invasive action filter that is safe w.r.t. *all* Gaussian primitives simultaneously, running at 15 Hz over hundreds of thousands of Gaussians while the splat is still training online; ships SplatBridge (ROS) for online GSplat mapping.
*How it differs:* it is the strongest existing evidence **against** your motivation. SAFER-Splat's whole point is that you do **not** need to reduce the primitive count — 10⁵-plus Gaussians at 15 Hz with a small memory footprint. A reviewer will ask directly: *if the state of the art already handles 10⁵ primitives in real time, what is the coreset for?* Your answer has to be that a CBF is a local reactive filter with no global reachability semantics, whereas you are compiling a global certified sandwich (M_safe ⊆ M_true ⊆ M_possible) whose cost is superlinear in a way theirs is not — **and you currently have no measurement of your compiler's asymptotic cost to back that up.** Fix that (see §4, §5).

**Let's Make a Splan: Risk-Aware Trajectory Optimization in a Normalized Gaussian Splat (SPLANNING)** — Jonathan Michaux, Seth Isaacson, Challen Enninful Adu, Adam Li, Rahul Kashyap Swayampakula, Parker Ewen, Sean Rice, Katherine A. Skinner, Ram Vasudevan. arXiv:2409.16915. https://arxiv.org/abs/2409.16915; https://roahmlab.github.io/splanning/ **[V]**
*What it does:* derives rigid-body collision *probability* in a radiance field starting from the rendering equation, upper-bounds it efficiently inside a 3DGS model, renormalizes the 3D Gaussians so the probabilities are well-defined, and uses the bound in a receding-horizon manipulator planner.
*How it differs:* this is the most direct challenge to your standing rule that "opacity/render alpha does NOT define physical collision." SPLANNING takes the opposite position and derives its semantics rigorously from the rendering model. You are not wrong — but **you now owe a paragraph explaining why you reject the probabilistic-occupancy reading**, and that paragraph will be read by the same reviewer pool. Also note: SPLANNING is a *probabilistic* guarantee and yours is a *set-based* one; that difference, not the opacity question, is your cleanest line of separation.

**FOCI: Trajectory Optimization on Gaussian Splats** — Mario Gomez Andreu, Maximum Wilder-Smith, Victor Klemm, Vaishakh Patil, Jesus Tordesillas, Marco Hutter (ETH Zurich RSL / Comillas). arXiv:2505.08510. https://arxiv.org/abs/2505.08510; code https://github.com/leggedrobotics/foci **[V]** (methodological details **[S]**)
*What it does:* represents **both** the environment and the robot as Gaussian splats and defines collision via the **overlap integral** between Gaussians, giving an orientation-aware, differentiable cost; optimizes trajectories in seconds over scenes with hundreds of thousands of Gaussians; deployed on ANYmal, explicitly targeting narrow passages that require the robot to rotate. The repo ships `demo/data/stonehenge.ply`, attributed to Splat-Nav's processed data — **so a Splat-Nav-compatible splat is one `git clone` away for you.**
*How it differs:* FOCI is the closest work to your *compiler's* geometric core — union-of-ellipsoids robot vs Gaussian scene, orientation-aware, narrow passages. Two decisive differences in your favour: (i) the overlap integral is a **soft, smooth** cost, not a certified separation — it produces good trajectories, not proofs, and it can trade a little penetration for a lot of smoothness; (ii) it optimizes a single trajectory rather than compiling a reusable global structure. Two differences against you: it is 3D, on real splats, on hardware, today.

**GaussNav: Gaussian Splatting for Visual Navigation** — Xiaohan Lei, Min Wang, Wengang Zhou, Houqiang Li. arXiv:2403.11625. https://arxiv.org/abs/2403.11625 **[V]**
*What it does:* builds a semantic Gaussian map for Instance ImageGoal Navigation, retaining geometry, semantics and object texture, then **converts it to a 2D BEV grid for the actual navigation** **[S]**.
*How it differs:* it is the existence proof that the 3DGS→2D-for-planning reduction is already standard practice — and that it is done *unsoundly*, by rasterizing to a grid. Your §5 importer is the sound version of GaussNav's throwaway step. That framing is worth more than treating GaussNav as a competitor.

**Rendering-oriented 3DGS compaction/compression (the wrong-objective baselines you must beat).**
- **3DGS.zip: A survey on 3D Gaussian Splatting Compression Methods** — M. Bagdasarian, P. Knoll, Y.-H. Li, F. Barthel, A. Hilsmann, P. Eisert, W. Morgenstern. arXiv:2407.09510; Computer Graphics Forum 2025 (EG STAR). https://arxiv.org/abs/2407.09510; live leaderboard https://w-m.github.io/3dgs-compression-survey/ **[V]**. Splits the field into *compression* (bytes) and *compaction* (primitive count) and benchmarks on MipNeRF360 / TanksAndTemples / DeepBlending / SyntheticNeRF with PSNR, SSIM, LPIPS, MB. **This is exactly the metric set you must argue is the wrong one, and the leaderboard gives you free baselines.**
- **LightGaussian** (~15× reduction, 200+ FPS) and **Mini-Splatting** (constrained primitive budget) **[S]** — the standard prune-and-distill and primitive-budget baselines.
- **NanoGS: Training-Free Gaussian Splat Simplification** — arXiv:2603.16103 **[S, post-cutoff, verify]**. Merges disjoint pairs of Gaussians using a **moment-matching cost** measuring how well a two-Gaussian mixture is approximated by one Gaussian, greedily, preserving the pair's mass. **This is the single most dangerous baseline for you**: it is training-free, geometric, merge-based, and structurally isomorphic to your hierarchy + macro step, differing only in that its admissibility criterion is moment-matching error rather than robot-conditioned mobility. If NanoGS at n=28 preserves your gate, your contribution collapses to "we picked a better merge criterion." Run it.
- Also in the same family and worth one line each **[S, post-cutoff, verify]**: GaussianPOP (arXiv:2602.06830, error-quantified simplification), Camera-Agnostic Pruning via Descriptor-Based Beta Evidence (arXiv:2603.21933), Optimized Minimal 3DGS (arXiv:2503.16924), SUCCESS-GS survey (arXiv:2512.07197), Splatwizard benchmark toolkit (arXiv:2512.24742).

**Verdict on 1(a):** no published work prunes or merges splats *against a planning-side invariant with a proof*. The gap is real. It is also obvious, and NanoGS-style geometric merging plus a Splat-Nav-style corridor check would close most of it in a weekend for someone who already has both codebases.

## 1(b) Coresets and geometric approximation with guarantees

**Geometric Approximation via Coresets** — Pankaj K. Agarwal, Sariel Har-Peled, Kasturi R. Varadarajan. *Combinatorial and Computational Geometry*, MSRI Publications vol. 52, Cambridge University Press, 2005, pp. 1–30. https://sarielhp.org/p/04/survey/survey.pdf **[V]**
*What it does:* establishes the coreset paradigm — compute a small Q ⊆ P that approximates P for a class of *extent measures*, then run an expensive exact algorithm on Q. The central object is the **ε-kernel**: a subset whose directional width approximates the full set's in every direction, of size O(1/ε^((d−1)/2)) in R^d, constructible in O(n + 1/ε^(d−3/2)) time. Companion journal paper: *Approximating extent measures of points*, JACM 2004.
*How it differs — and this is the uncomfortable part:* **your outer macro E+ is an ε-kernel in all but name.** You certify a support-function inequality over 4096 sampled directions, subtract a Lipschitz half-step to get all directions, then inflate radially. That is precisely "approximate the directional width / support function uniformly over S¹ with a bounded one-sided error," which is the defining property of an ε-kernel, computed here by MVEE rather than by the standard fatten-and-grid construction. The literature already knows that (i) an affine-invariant "make it fat" preprocessing exists, (ii) O(1/√ε) directions suffice in the plane, and (iii) the construction is stable under perturbation (*Stability of ε-kernels*, arXiv:1003.5874 **[S]**). **You should cite this, claim the connection yourself, and position your contribution as the *admissibility criterion*, not the outer approximation.** If a reviewer discovers the ε-kernel connection before you concede it, the paper is dead.

**Coresets and Sketches** — Jeff M. Phillips. arXiv:1601.00617; Chapter 48/49, *Handbook of Discrete and Computational Geometry*, 3rd ed., CRC Press, 2017. https://arxiv.org/abs/1601.00617 **[V]**
*What it does:* the modern reference taxonomy — shape-fitting, density estimation, high-dimensional vectors, matrices, clustering — with the standard existence/size results.
*How it differs:* it is a taxonomy of coresets for *static extent and clustering objectives*. There is no entry for "coreset for a topological invariant of the eroded complement of the input, parameterized by a query body." That absence is your novelty claim's best evidence — **and also its best refutation risk**, because the absence may simply mean nobody found the object interesting, not that nobody could construct it.

**MVEE background:** Khachiyan's barycentric coordinate descent for minimum-volume enclosing ellipsoids (1996), with the standard modern analysis in **Todd & Yıldırım, "On Khachiyan's algorithm for the computation of minimum-volume enclosing ellipsoids," Discrete Applied Mathematics 155(13):1731–1744, 2007**, DOI 10.1016/j.dam.2007.02.013 **[V]**. Your design decision — that MVEE quality affects tightness but never soundness, because containment is re-certified independently — is correct and is worth one explicit sentence in the paper; it is the kind of thing reviewers assume you got wrong.

## 1(c) Map/mesh simplification with topology or clearance guarantees

**Topological Persistence and Simplification** — Herbert Edelsbrunner, David Letscher, Afra Zomorodian. *Discrete & Computational Geometry* 28:511–533, 2002 (FOCS 2000). https://pub.ista.ac.at/~edels/Papers/2002-04-TopologicalPersistence.pdf **[V]**
*What it does:* defines persistence — classifying topological features as signal or noise by lifetime within a filtration — and gives algorithms for computing it and for simplifying based on it.
*How it differs — and this is your sharpest exposure:* **your merge-admissibility clause is a persistence criterion in disguise.** "The (#components, #holes) signature of `erode(free, r)` is unchanged at probe radii bracketing every critical radius, with critical radii found by bisection over r ∈ [b, a]" is exactly: *the H₀ and H₁ barcodes of the erosion filtration of free space, restricted to the window [b, a], are unchanged*. The erosion-by-radius filtration is the standard clearance/medial-axis filtration. Your bisection for critical radii is a search for the barcode's birth/death events. A computational-topology reviewer will see this on the first read. **Own it: cite Edelsbrunner–Letscher–Zomorodian, say "we compute a restricted persistence signature of the clearance filtration," and make the contribution the *use* of it as a merge gate, not the invariant.**

**Stability of Persistence Diagrams** — David Cohen-Steiner, Herbert Edelsbrunner, John Harer. *Discrete & Computational Geometry* 37:103–120, 2007 **[V]**.
*Why it hurts:* the stability theorem says small perturbations of the input produce boundedly small changes in the persistence diagram. Applied to your two clauses, this suggests the ε-intrusion clause **already implies** a bounded change in the topology signature — i.e. your disjunction ("ε-intrusion **OR** unchanged signature") may have a redundant branch, or the signature clause is doing work only in the regime where the intrusion bound is loose. A reviewer will ask you to characterize *exactly which merges the signature clause admits that the ε clause rejects*, with a measured count. **You should be able to state that number. Right now you cannot.** If the answer is "almost none," delete the clause and the paper gets simpler and stronger; if it is "most of the compression," that is a headline result you are currently not claiming.

**Alpha shapes** — Herbert Edelsbrunner, David Kirkpatrick, Raimund Seidel, "On the shape of a set of points in the plane," *IEEE Transactions on Information Theory* 29(4):551–559, 1983, DOI 10.1109/TIT.1983.1056714 **[V]**; 3D extension: Edelsbrunner & Mücke, "Three-dimensional alpha shapes," *ACM TOG* 13(1):43–72, 1994, DOI 10.1145/174462.156635 **[V]**.
*How it differs:* alpha shapes give a *parameterized family* of shapes from a point set with known homotopy relationships to the underlying space — the canonical "geometry at a scale" construction. Your macro ellipses are a different parameterization of the same idea (scale = robot radius rather than α). Difference: alpha shapes reconstruct the shape; you *conservatively over-approximate the obstacle and under-approximate free space*, which is the safety-relevant asymmetry alpha shapes do not provide.

**Topology-Preserving Terrain Simplification** — arXiv:1912.03032 **[S]**; **Topology-preserving graph coarsening via elementary collapse** — PVLDB 17, 2024, https://www.vldb.org/pvldb/vol17/p4760-li.pdf **[V]**.
*How they differ:* both preserve homotopy type of a *fixed* object. Yours must preserve the topology of a *one-parameter family* (erosion by r over [b, a]) — strictly harder, and this is a legitimate, statable technical distinction. Use it.

**Medial axis / GVD roadmaps.** Standard: the generalized Voronoi diagram of free space is the maximum-clearance roadmap; MAPRM (Wilmarth, Amato, Stiller, ICRA 1999) retracts samples onto it without computing it. **Sparse 3D Topological Graphs for Micro-Aerial Vehicle Planning** (Oleynikova et al., arXiv:1803.04345) **[S]** builds a sparse GVD-based skeleton from an ESDF for planning.
*How they differ:* a GVD/skeleton *summarizes free space* and its topology is exactly what you are preserving — so a reviewer will propose the obvious alternative pipeline: **compute the GVD of the splat map once, and coarsen anything that does not touch a GVD edge with clearance in [b, a].** That is a strong, cheap, well-understood baseline and you must run it (§4). The counter-argument in your favour: a GVD is a structure over *free* space, which for a non-circular robot in SE(2) is orientation-dependent, and 2D GVD clearance is a circular-robot notion. That is a real defence — make it explicit and quantitative.

**Persistent homology for planning:** Ghrist and collaborators, *Persistent Homology for Path Planning in Uncertain Environments* (see https://www2.math.upenn.edu/~ghrist/preprints/persistence_planning.pdf) **[V]**; Pokorny, Hawasly, Ramamoorthy, "Topological trajectory classification with filtrations of simplicial complexes and persistent homology," *IJRR* 35(1–3), 2016 **[V]**; Bhattacharya et al. on H-signatures and homotopy-class planning **[S]**.
*How they differ:* these use topology to *classify or constrain paths*. None uses topology as an **admissibility gate on map compression**. That specific move is, as far as I can find, unpublished — and it is the single defensible novelty in the coreset layer.

## 1(d) C-space obstacle approximation and certified planning

**Efficient Path Planning in Narrow Passages for Robots With Ellipsoidal Components** — Sipu Ruan, Karen L. Poblete, Hongtao Wu, Qianli Ma, Gregory S. Chirikjian. *IEEE Transactions on Robotics* 39(1):110–127, Feb 2023. arXiv:2104.04658. DOI 10.1109/TRO.2022.3187818 (**verify the DOI digits before citing** — confirmed volume/issue/pages, not the DOI suffix). Code: https://github.com/ChirikjianLab/hrm **[V]**
*What it does:* motion planning built on **closed-form Minkowski sum and difference between an ellipsoid and a general convex-differentiable obstacle (superquadrics)**; Highway RoadMap (HRM) for SE(2) and SE(3) rigid bodies; Prob-HRM for articulated bodies; C++ released; benchmarked as beating sampling-based planners specifically in narrow passages.
*How it differs from your compiler — honestly, barely:*
| | HRM (2023) | Your compiler |
|---|---|---|
| C-obstacle | closed-form Minkowski sum/difference, ellipsoid vs superquadric | closed-form support function, ellipse vs ellipse |
| Free-space parameterization | sweep lines → free segments → highway layers, glued across orientation | inner/outer convex sandwiches glued across orientation into slabs |
| Orientation handling | discrete orientation layers with bridge C-layers | certified continuous θ slabs |
| Output | roadmap | M_safe ⊆ M_true ⊆ M_possible + REACHABLE / UNREACHABLE / UNKNOWN |
| Certificate | free segments are exact by construction | explicit two-sided sandwich, independent path re-verification |
| Code | public C++, benchmarked | not public |

Your genuine deltas are: (i) **certified continuous coverage in θ** rather than discrete layers plus heuristic bridges; (ii) an **explicit UNREACHABLE verdict from a certified global cut**, which HRM does not emit; (iii) the ellipse-vs-ellipse specialization is closed-form where HRM's ellipsoid-vs-superquadric is closed-form-with-numerics. Those are real, and they are *increments on HRM*, not a new paradigm. **Any paper that does not run against the released HRM C++ on HRM's own narrow-passage benchmarks will be desk-rejected by anyone who knows the field.**

**SE(2) Navigation Mesh** — Shuyang Shi, Kaixian Qu, Changan Chen, Ines Kast, Yuntao Ma, Marco Hutter (ETH Zurich RSL). arXiv:2607.01454, submitted 1 July 2026. https://arxiv.org/abs/2607.01454 **[V for existence/authors/abstract; S for details — post-cutoff]**
*What it does:* a polygonal representation of traversable regions encoding **yaw-dependent** traversability; evaluates traversability with footprint masks; builds a graph over **yaw-specific layers** with explicit translational and rotational connectivity; motivated exactly by "traditional navmeshes assume yaw-invariant traversability, unsuitable for non-circular robots in constrained spaces"; deployed on a real robot with real-time online updates.
*How it differs — and why this is your most urgent competitive read:* this is your problem statement, from ETH, two months ago, on hardware. It is **discrete in yaw** and its polygonal abstraction is **not certified** — those are your two openings. But it also *already does the polygonal simplification of free space for a non-circular robot*, which is 80% of what "robot-conditioned mobility coarsening" means to a reader. **Read this paper today.** If its polygon simplification tolerance is chosen from the footprint's inscribed/circumscribed radii, your novelty claim is in serious trouble and you need to know that before you write, not after.

**Configuration Space Distance Fields for Manipulation Planning (CDF)** — Yiming Li, Yan Zhang, Amirreza Razmjoo, Sylvain Calinon. arXiv:2406.01137; RSS 2024, https://www.roboticsproceedings.org/rss20/p131.pdf **[V]**
*What it does:* learns a distance field in *joint* space rather than task space (distance to collision measured in radians, not metres), with a neural (MLP) representation; demonstrated on planar obstacle avoidance and a 7-axis Franka. Follow-ups: **Neural Configuration-Space Barriers** (arXiv:2503.04929) **[S]**, **CDFlow** (arXiv:2509.13771) **[S]**, CSSDF-Net (arXiv:2603.18669-adjacent) **[S, post-cutoff]**.
*How it differs:* CDF is *learned, approximate, and uncertified* — it has no soundness guarantee at all. That is a clean separation and you should say it in one sentence. But note the reviewer's counter: CDF scales to 7-DoF and arbitrary scenes; you have 3-DoF and 484 ellipses. Certification bought with a 4-orders-of-magnitude scale gap is a hard sell.

**IRIS: Computing Large Convex Regions of Obstacle-Free Space Through Semidefinite Programming** — Robin Deits, Russ Tedrake. WAFR 2014, *Springer Tracts in Advanced Robotics* 107, 2015, DOI 10.1007/978-3-319-16595-0_7; code https://github.com/rdeits/iris-distro **[V]**. Successors: IRIS-NP (arXiv:2303.14737), Fast Iterative Region Inflation (arXiv:2403.02977), and **Graphs of Convex Sets** motion planning (Marcucci et al., *Science Robotics* 2023, DOI 10.1126/scirobotics.adf7843) **[V]**.
*How they differ:* IRIS/GCS produce **inner** convex approximations of free space with rigorous separation — i.e. exactly your M_safe side, in a mature, well-tooled form, in arbitrary dimension. **A reviewer will ask why you did not just run IRIS-NP in SE(2) and get certified convex free regions directly.** Your answer must be: IRIS gives you M_safe only, never M_possible, so it can never emit a certified UNREACHABLE; and it is a per-query/per-seed construction, not a compiled scene summary. Have that answer ready with a measurement behind it.

**Path Planning in Complex Environments with Superquadrics and Voronoi-Based Orientation** — Lin Yang, Ganesh Iyer, Baichuan Lou, Sri Harsha Turlapati, Chen Lv, Domenico Campolo. arXiv:2411.05279 **[V]**.
*What it does:* expands superquadric obstacles to **eliminate impassable passages**, uses Voronoi hyperplanes for maximum clearance, and **aligns the robot's long axis with the passage direction** from the hyperplane normal.
*How it differs:* note carefully — "expanding obstacles to eliminate impassable gaps while preserving feasible ones" is a *robot-conditioned scene modification with a mobility-preservation intent*, published in Nov 2024. It is heuristic, not certified, and it modifies rather than compresses. But the **idea** — grow obstacles by a robot-derived amount specifically so that gaps the robot cannot use disappear while gaps it can use survive — is the conceptual core of your admissibility criterion, already in print. **Cite it, distinguish on "certified + compressive vs heuristic + non-compressive," and do not let a reviewer find it first.**

## 1(e) Adaptive / multiresolution occupancy for planning

**OctoMap: an efficient probabilistic 3D mapping framework based on octrees** — Armin Hornung, Kai M. Wurm, Maren Bennewitz, Cyrill Stachniss, Wolfram Burgard. *Autonomous Robots* 34(3):189–206, 2013. DOI 10.1007/s10514-012-9321-0. https://octomap.github.io/ **[V]**
*What it does:* probabilistic octree occupancy mapping that explicitly represents **occupied, free, and unknown** space, with lossless octree pruning (collapsing uniform children) for compactness.
*How it differs — and read this one carefully, because it is the "so what" attack:* OctoMap's occupied/free/unknown trichotomy **is a sandwich**: free ⊆ true-free ⊆ (free ∪ unknown). Its pruning is lossless w.r.t. that sandwich. So "we emit M_safe ⊆ M_true ⊆ M_possible with lossless coarsening" describes OctoMap at the level of abstraction a reviewer will first apply. Your actual differences: OctoMap's coarsening is (i) **axis-aligned and grid-quantized**, so it cannot be tight on an oblique wall; (ii) **not robot-conditioned** — pruning is decided by child uniformity, never by whether the robot could fit; (iii) **workspace-only** — orientation enters only by inflating to the circumscribed radius, which is precisely the conservatism that kills non-circular robots in narrow passages. Those three points are your answer, and you must make them *quantitatively*, which means running an octree baseline (§4).

**Selective Densification for Rapid Motion Planning in High Dimensions with Narrow Passages** — Lu Huang, Lingxiao Meng, Jiankun Wang, Xingjian Jing. arXiv:2507.15710, submitted 21 July 2025; IEEE T-ASE (Xplore doc 11153478) **[V]**.
*What it does:* multi-resolution sampling — search mostly over sparse samples, switch to dense samples only where the sparse graph fails to yield a feasible path; SE(2), SE(3), R¹⁴ and a Franka in a constrained workspace.
Predecessors: **Fast Planning Over Roadmaps via Selective Densification** (arXiv:2002.04941) and Choudhury et al., *Densification Strategies for Anytime Motion Planning over Large Dense Roadmaps* (arXiv:1611.00111) **[V]** — a sequence of increasingly dense subgraphs with layer edges between identical configurations.
*How they differ:* these adapt resolution **in the planner's search**, reactively, with no guarantee about what the coarse level omits; you adapt resolution **in the map**, once, with a certificate. That is a clean and defensible distinction. The uncomfortable adjacency: their coarse level failing → refine locally is operationally equivalent to your UNKNOWN verdict → look again, and their pipeline achieves it without needing any of your machinery. **A reviewer will say the certificate buys you the ability to answer UNREACHABLE, and nothing else.** Be ready to say what UNREACHABLE is *worth* — which, as far as I can tell, you have never argued for anywhere.

**Communication-Aware / Task-Driven Map Compression** — arXiv:2503.10843, 2506.20579, 2309.13451, 2403.14780, 2509.07655 **[S]**.
*What they do:* rate-distortion formulations for compressing maps for *transmission*, choosing when to communicate, which regions to include, at what resolution, driven by a downstream navigation task.
*How they differ:* the objective is bandwidth under a task-utility distortion, and the guarantee is statistical/expected, not set-theoretic. **But the framing "task-driven map compression, resolution chosen per region by downstream planning utility" is already the title of a small literature.** Your differentiator is the word *certified*, and only that word.

---

# 2. NOVELTY — ruthless assessment

## 2.1 Is "derive the minimal robot-conditioned scene geometry that provably preserves certified mobility topology" new?

Decomposed into its four claims:

| Component | Novel? | Prior art that already covers it |
|---|---|---|
| Conservative outer macro-primitive with certified containment over all directions | **No** | ε-kernels / extent-measure coresets (Agarwal–Har-Peled–Varadarajan 2005). Your MVEE + 4096 directions − Lipschitz half-step + radial inflation is a correct but standard construction. |
| Preserving the topology of a filtration under simplification | **No** | Persistence-based simplification (Edelsbrunner–Letscher–Zomorodian 2002); stability (Cohen-Steiner–Edelsbrunner–Harer 2007). |
| Growing obstacles by a robot-derived amount to kill unusable gaps while preserving usable ones | **No** | Yang et al. arXiv:2411.05279 (superquadric expansion + Voronoi), heuristically; classical C-obstacle inflation, universally. |
| **Using a robot-conditioned topological invariant of the erosion filtration as an admissibility gate on primitive merging, with a two-sided certificate carried into a planner that emits certified UNREACHABLE** | **Yes, narrowly** | I found nothing. This is the contribution. It is one clause of one algorithm. |

**Blunt reading:** the novelty is *the gate*, not the coreset, not the certification, not the robot conditioning. That is enough for a paper only if the gate is shown to buy something no cheaper gate buys — which is precisely what the missing surface-only × robot-conditioned cell would show, and precisely what the geometry-only control alone does *not* show (geometry-only is a strawman: it has no gate at all; the real competitors are NanoGS's moment-matching gate, an ε-intrusion-only gate, and a GVD-clearance gate).

## 2.2 The three closest works, and the smallest change that subsumes you

**1. Ruan, Poblete, Wu, Ma, Chirikjian — HRM, T-RO 2023.**
*Smallest subsuming change:* add one preprocessing stage to the released C++ — cluster the superquadric obstacles, replace each cluster by its minimum-volume enclosing superquadric, and accept the replacement iff the connectivity of the sweep-line free segments is unchanged at the robot's semi-minor and semi-major radii. That is roughly half a page of method and one figure in an HRM journal extension. **HRM already has the sweep-line free-space parameterization that makes the connectivity check nearly free.** This is the most likely way you get scooped, and the group that would do it is the group whose paper you already collided with.

**2. Chen, Shorinwa, Yu, Schwager et al. — Splat-Nav / SAFER-Splat.**
*Smallest subsuming change:* run any off-the-shelf training-free splat merger (NanoGS) but replace its render-fidelity acceptance test with "the minimum clearance of the Splat-Plan polytope corridor between a fixed set of start/goal pairs is unchanged," proving containment with the same ellipsoid level-set bound they already use. They have the corridor machinery, the ellipsoid bound, the scenes, and the hardware. **This is a two-week project for that lab.**

**3. Shi, Qu, Chen, Kast, Ma, Hutter — SE(2) NavMesh, arXiv:2607.01454 (July 2026).**
*Smallest subsuming change:* choose the polygon-simplification tolerance in each yaw layer from the footprint's inscribed radius, and verify that each layer's connected components and the inter-layer rotational edges are unchanged after simplification. One paragraph. They already have yaw layers, footprint masks, polygonal abstraction, and a real robot. **Check whether they have already done this. If they have, the coreset layer is dead as a standalone contribution and must be repositioned as "the certified version of NavMesh simplification."**

## 2.3 Is "coresets are ordered by robot size" already known?

**Yes — it is a corollary of morphological monotonicity, and you can derive the exact asymmetry you measured in three lines.**

Let `F` be free space, `E(r)` the closed disc of radius `r`, and `erode(F, r) = F ⊖ E(r)`. Erosion is monotone decreasing in `r`: `r₁ ≤ r₂ ⟹ erode(F, r₂) ⊆ erode(F, r₁)` (Matheron/Serra mathematical morphology; equivalently, C-obstacle monotonicity under robot inclusion: `A ⊆ B ⟹ O ⊖ A ⊆ O ⊖ B`, so `F_free(B) ⊆ F_free(A)`).

Now let a merge add obstacle mass, i.e. produce `F' ⊆ F`. Say the merge **seals at radius r₀** if two components of `erode(F, r₀)` that were connected become disconnected in `erode(F', r₀)`.

**Claim (sealing propagates upward in r).** If a merge seals at `r₀`, it seals at every `r ≥ r₀`.
*Proof sketch:* an `r`-clearance path is in particular an `r₀`-clearance path for `r ≥ r₀`; if no `r₀`-clearance path survives in `F'`, no `r`-clearance path survives either. ∎

**Corollary (the ordering, with its direction).** A coreset whose admissibility was verified at probe radii covering `[b_L, a_L]` is automatically admissible for any robot whose radius window `[b_S, a_S]` satisfies `a_S ≤ a_L` — because damage at any `r ≤ a_L` would have manifested at `a_L` and been rejected. The converse fails: verification over `[b_S, a_S]` says nothing about `r > a_S`.

**Therefore:** a coreset built for the **large** robot (a=0.40) is adequate for the **small** one (a=0.20); a coreset built for the small robot is **not** adequate for the large one. **This is exactly your reported two-door adequacy matrix.** You did not discover an empirical ordering; you ran a confirmation of a theorem you have not yet stated.

**Three consequences, in increasing order of importance:**

1. **Stop presenting the adequacy matrix as a finding.** Present the corollary as a proposition with the matrix as its verification. A reviewer who sees a 2×2 empirical matrix "revealing" a monotonicity fact will conclude you do not understand your own construction.

2. **The ordering is by a scalar, which weakens "robot-conditioned."** If the gate depended only on the erosion clause, conditioning collapses to a single number, `r_max = a`. Everything eccentricity-dependent enters only through the *other* clause — the ε-intrusion depth, whose metric value you derive from the gate-angle tolerance via `dθ/dr = r / (sin t cos t (a²−b²))`. So the true partial order is on the pair **(a, ε_metric(a,b))**, not on "robot size," and it is a **partial** order, not a total one. Your two-door pair (0.20,0.10) vs (0.40,0.25) is *comparable in both coordinates* — you tested the easy case. **The falsifiable prediction that would actually be informative: construct an incomparable pair (a small but highly eccentric robot vs a larger but rounder one) and show neither coreset subsumes the other.** If it turns out they are always comparable, the "coreset lattice" story is much weaker than it sounds.

3. **The corollary is your amortization argument, and you should lead with it.** Build **one** coreset at the largest `a` in your robot fleet; it is certified for every smaller robot for free. That converts coarsening from a per-(scene, robot) cost into a per-scene cost, which is the only thing that makes 15–27 s of coarsening defensible. **This is the strongest positive result in the whole project and it is currently buried as a caveat about an "asymmetric adequacy matrix."**

## 2.4 Does the certified sandwich add anything over conservative voxel/octree inflation?

**Partly, and less than the framing implies.**

What it does **not** add: the sandwich structure itself. OctoMap's occupied/free/unknown is a sandwich, its pruning is lossless w.r.t. it, and inflating obstacles by the robot's circumscribed radius yields a sound inner approximation of free space at any finite resolution. "We produce a two-sided approximation" is not a differentiator against a 2013 paper.

What it **does** add, in decreasing order of defensibility:

1. **Continuous θ.** A voxel/octree pipeline handles orientation either by circumscribed-radius inflation (badly conservative — it cannot pass any gate narrower than 2a, so it seals every genuinely interesting narrow passage) or by a stack of inflated grids at sampled yaws (SE(2) NavMesh's approach), which is sound only at the sampled yaws and needs an unproven interpolation argument between them. Your certified slab covers a *continuous* θ interval. **This is the one place where you are strictly stronger than everything in §1(e) and §1(d) except HRM.**
2. **No (x,y) discretization error on oblique geometry.** A grid must choose between conservatism and resolution on a wall at 37°; a support function does not.
3. **A certified global cut → UNREACHABLE.** An octree can also emit "no path exists through free ∪ unknown," which is a certified cut. So this is a *smaller* delta than it sounds; what you add is that your cut is exact in (x,y,θ) rather than resolution-limited.

**What you must therefore measure (and have not):** an octree/voxel baseline at matched primitive budget, with orientation handled both ways (circumscribed inflation, and an N-layer yaw stack), reporting gate error and verdict agreement. If a 28-leaf clearance-adaptive quadtree with 16 yaw layers recovers the gate to 1e-3, the "certified sandwich" contribution is presentational.

## 2.5 Blunt verdict

**Is this a paper?** Not yet, and not as a T-RO paper. Here is the decomposition:

- **The compiler alone: no.** Subsumed by HRM in scope and beaten by it in maturity (SE(3), articulated bodies, superquadrics, released C++, published benchmarks). Your continuous-θ certificate and UNREACHABLE verdict are increments, publishable only as part of a larger claim.
- **The coreset alone, on synthetic scenes: no.** One scene family, one robot for the headline, an artifact-inflated compression ratio, a strawman control, and a guarantee that is one-sided where the claim is two-sided.
- **The coreset on real 3DGS, against real baselines, with the monotonicity theorem stated and proved: yes — RA-L, or ICRA.** That is the honest target. RSS wants a bigger idea than this; T-RO wants either the 3D generalization or a substantially deeper theory. ICRA/RA-L is the right venue for "here is a certified, robot-conditioned map reduction that makes certified SE(2) planning tractable on real splat maps."
- **T-RO becomes reachable if and only if** you extend to SE(3) (or give a real reason SE(2) suffices), run on ≥3 real scene families at ≥10⁵ splats, beat HRM's C++ on HRM's benchmarks, and prove the two-sided preservation theorem rather than measuring one side.

**Minimum result set for an ICRA/RA-L submission:**
1. A stated and proved **two-sided** preservation proposition: under the admissibility criterion, (i) M_safe(coreset) ⊆ M_true — soundness, currently only measured; and (ii) every query the uncompressed compiler answers REACHABLE, the coreset also answers REACHABLE — completeness preservation, currently not even claimed. Plus the monotonicity corollary of §2.3.
2. Real 3DGS input via the §5 importer, ≥3 scenes, ≥10⁵ splats each, with the κ-calibration against a ground-truth mesh.
3. Compression at **zero verdict change** against ≥5 baselines at matched budget: random, opacity/LightGaussian, voxel/octree, clearance-adaptive octree, NanoGS moment-matching, geometry-only, and the surface-only filter alone.
4. HRM (released C++) as the external planner baseline on its own narrow-passage benchmarks, timing normalized to absolute seconds, not to your own uncompressed path.
5. The amortization curve over robots and scenes, with the monotonicity corollary as the reason the curve is favourable.
6. A property-based randomized soundness campaign at N ≥ 10⁵ with zero violations, reported with a rule-of-three bound — replacing "~40 arms."

---

# 3. BENCHMARKS — what this could actually run on

Ranked by effort-to-value. "Effort" = engineering distance from a working SE(2) pipeline; "value" = credibility with a robotics reviewer.

### Tier 1 — do these

**1. InteriorGS (Manycore Research / SpatialVerse).** https://github.com/manycore-research/InteriorGS · https://huggingface.co/datasets/spatialverse/InteriorGS **[V — page fetched]**
- **What:** 1,000 indoor scenes, 80+ environment types (homes, stores, wedding halls, museums), each with a **compressed 3DGS PLY** (position, covariance, opacity, SH), 554K instance-level 3D oriented bounding boxes over 755 categories, **1024×1024 occupancy maps** (white=free, black=occupied, grey=unknown), and **JSON floorplans** with wall geometry and door/window openings. XYZ = (Right, Back, Up), **metric units (metres)**. v2.0 adds floorplans for all 1,000 scenes.
- **Licence:** custom "InteriorGS License" (ToU PDF); HuggingFace-hosted, no email gatekeeping.
- **Why it is #1 by a wide margin:** it is the only dataset that hands you, simultaneously, (i) native 3DGS primitives, (ii) a metric scale, (iii) an independent occupancy ground truth to validate your importer against, and (iv) an explicit list of **doors** — i.e. labelled narrow passages, which is exactly your gate structure at scale. Your single-door synthetic scene generalizes to "the door openings in the floorplan JSON," and each becomes an analytic-ish gate with known width.
- **Caveats to check before committing:** splat counts per scene are not stated on the repo page (**verify**); the scenes are synthetic-CG-derived rather than scanned, so they will have clean surfaces and *may reproduce the volume-filled-wall artifact* that inflated your 17.3× — check whether walls are hollow shells or solid before you trust any compression number from this dataset.

**2. Splat-Nav / FOCI released scenes — Stonehenge, Statues, Flightroom, Old Union.** https://github.com/chengine/splatnav · https://github.com/leggedrobotics/foci (ships `demo/data/stonehenge.ply`) **[V]**
- **Scale:** Stonehenge 116K dense / 12K sparse, Statues 201K / 18K, Flight 281K / 4K **[S — verify in Table III]**.
- **Licence:** repository licences, permissive; **zero acquisition friction — you can have `stonehenge.ply` in five minutes.**
- **Value:** direct comparability with the two closest planning-on-splats papers, and a scene (Stonehenge) that is literally a ring of pillars with gaps — a genuine narrow-passage structure with a natural 2D slice.
- **Caveat:** these are drone/3D scenes; Stonehenge has no floor-plan semantics and the "passages" are between free-standing pillars, so an SE(2) slice is meaningful but the task is somewhat artificial for a ground robot. **Use it as the smoke test and the comparability anchor, not the headline.**

### Tier 2 — worth the effort once Tier 1 works

**3. Replica.** Straub et al., *The Replica Dataset: A Digital Replica of Indoor Spaces*, arXiv:1906.05797. https://github.com/facebookresearch/Replica-Dataset **[V]**
- **What:** 18 photorealistic room- and building-scale reconstructions, dense mesh + HDR textures + per-primitive semantic class/instance + **explicitly annotated planar mirror and glass reflectors**.
- **Licence:** Replica research licence (non-commercial); download script from the repo. Habitat ToU applies when used through Habitat (https://aihabitat.org/terms-of-use/).
- **Why it matters uniquely:** the **ground-truth mesh is the instrument that calibrates κ** (§5). You cannot honestly pick a level-set parameter without measuring one-sided Hausdorff distance from a true surface to the κ-union, and Replica is the cleanest place to do that. The annotated mirrors/glass additionally let you *measure* the phantom-geometry failure mode instead of hand-waving it.
- **Caveat:** you must train the 3DGS yourself (gsplat/Nerfstudio from Replica renders); narrow passages are limited to doorways and furniture gaps. Small scenes.

**4. ScanNet++.** Chandan Yeshwanth, Yueh-Cheng Liu, Matthias Nießner, Angela Dai. arXiv:2308.11417; ICCV 2023. https://scannetpp.mlsg.cit.tum.de/ **[V]**
- **What:** 460 scenes; sub-millimetre laser-scan geometry; 280K registered 33 MP DSLR images; 3.7M iPhone RGB-D frames; open-vocabulary semantics; a real **novel-view-synthesis benchmark**, so 3DGS training recipes are standard.
- **Licence:** ScanNet++ Terms of Use, signup required.
- **Value:** the best geometry in the field, which makes it the strongest possible κ-calibration and the strongest possible refutation venue for "your certificate is only relative to the splats." Cluttered real rooms give genuine narrow passages (between furniture, not just doorways).
- **Effort:** signup + 3DGS training + floor extraction. Real work, high payoff.

### Tier 3 — only if a reviewer demands embodied-AI comparability

**5. HM3D.** Ramakrishnan et al., *Habitat-Matterport 3D Dataset (HM3D): 1000 Large-scale 3D Environments for Embodied AI*, arXiv:2109.08238, NeurIPS 2021 Datasets & Benchmarks. https://github.com/facebookresearch/habitat-matterport3d-dataset **[V]**
- 1,000 building-scale reconstructions, 112.5K m² navigable, 38 countries; 20–85% higher visual fidelity and 34–91% fewer reconstruction artifacts than MP3D/Gibson/Replica/ScanNet. Habitat ToU; academic access.
- **Value:** the most and hardest genuine narrow passages (real cluttered homes, doorways, corridors) and the standard navigation benchmark, so results are legible to the embodied-AI community.
- **Killer caveat:** **multi-floor buildings.** Your single height-band assumption breaks immediately; you need per-floor segmentation before the importer runs. Plus 3DGS must be trained from Habitat renders. Highest effort in the list.

**6. Matterport3D / Gibson / iGibson.** MP3D: EULA by email to matterport3d@googlegroups.com under the Matterport academic-use EULA; derived trained models distributed CC BY-NC-SA 3.0 US. Gibson/iGibson: licence agreement form → download URL; 500+ Matterport-captured homes and offices. **[V]**
- Value now mostly historical comparability. Skip unless a reviewer names them.

**7. 3DGS-native navigation simulators — NavGSim (arXiv:2603.15186), NVSim (arXiv:2510.24335), SplatSearch (arXiv:2511.12972), Splatblox (arXiv:2511.18525), LagMemo (arXiv:2510.24118). [S, all post-cutoff — availability unverified]**
- NavGSim is directly relevant: it "projects 3D models onto multiple horizontal Z-planes at fixed height intervals, where each plane contains a 2D Gaussian slice," combining slices into a 2D collision map **[S]** — i.e. **someone has already published the 3DGS→2D-slice reduction for navigation.** Read it before writing §5's related work. It is almost certainly *unsound* (slices, not shadows — see §5.2), which is your opening, but you must engage it.

### Ranking summary

| Rank | Dataset | 3DGS native | Metric scale | 2D floorplan/slice | Real narrow passages | GT mesh for κ | Access friction | Effort→Value |
|---|---|---|---|---|---|---|---|---|
| 1 | **InteriorGS** | ✅ PLY | ✅ m | ✅ JSON + occupancy PNG | ✅ labelled doors | ⚠️ CG-derived | low (HF) | **best** |
| 2 | **Splat-Nav/FOCI scenes** | ✅ PLY | ⚠️ verify | ❌ derive | ⚠️ pillar gaps | ❌ | none | **best smoke test** |
| 3 | **Replica** | train it | ✅ | derive | ⚠️ doorways | ✅ + mirrors/glass | low | high |
| 4 | **ScanNet++** | train it | ✅ laser | derive | ✅ clutter | ✅ best | medium (signup) | high |
| 5 | **HM3D** | train it | ✅ | derive per floor | ✅✅ best | ⚠️ mesh | medium | medium (multi-floor) |
| 6 | MP3D / Gibson | train it | ✅ | derive | ✅ | ⚠️ | high (email EULA) | low |
| 7 | NavGSim / NVSim / etc. | ✅ | ? | possibly ready-made | ? | ? | unverified | **read first, decide after** |

---

# 4. VALIDATION PROTOCOL

## 4.1 How the prior work validates compression/pruning claims

**Rendering-side (3DGS.zip and everything it indexes).** Independent variable: a size or count budget. Dependent: PSNR / SSIM / LPIPS and MB, on MipNeRF360, TanksAndTemples, DeepBlending, SyntheticNeRF. Presentation: rate-distortion curves (quality vs MB), a sortable leaderboard, per-scene tables. Ablations: component-wise (pruning alone, SH quantization alone, entropy coding alone). **The methodological lesson worth stealing: they always plot a curve, never a point.** Your entire result set is currently points (n=28, n=12–15). A single operating point is not a compression result; the field expects the whole trade-off frontier.

**Planning-side (Splat-Nav).** 8 scene variants (4 scenes × dense/sparse), **100 start/goal pairs per scene**, three baselines (point-cloud planner, RRT\*, NeRF-Nav), metrics on safety (collision/violation), path quality, and computation time **[S]**. **Lesson: 100 randomized queries per scene is the floor for credibility. You report one query.**

**Planning-side (SAFER-Splat).** Reports throughput (15 Hz), primitive count handled (hundreds of thousands), memory footprint, GPU utilization fraction, and intervention rate ("minimally invasive" = how often the filter modifies the nominal action) **[S]**. **Lesson: the field measures conservatism as an explicit metric.** Your analogue is the UNKNOWN rate, which you never report.

**Planning-side (FOCI, HRM, Selective Densification).** Success rate and planning time vs a narrow-passage difficulty parameter, against sampling-based baselines; HRM specifically sweeps passage width **[S]**. **Lesson: sweep the passage width.** You have exactly one door width (0.60 m). The gate width is your natural difficulty axis and you are not using it.

**Topology-side.** Persistence-based work validates by bottleneck/Wasserstein distance between diagrams and by stability bounds. **Lesson: you already compute a topological signature; report the *distance between diagrams*, not just an equality flag.** Equality is binary and uninformative near the boundary; distance tells a reviewer how close you came to failing.

## 4.2 The protocol this project should run

### Independent variables
- **n** — target support count, swept logarithmically from the full count down to failure (this produces the curve you are missing).
- **Robot** — a grid over (a, b), deliberately including **incomparable pairs**: e.g. (a=0.50,b=0.20) eccentric-small vs (a=0.55,b=0.45) round-large, so that a is nearly equal but ε_metric(a,b) differs sharply.
- **ε** — gate-angle tolerance, ≥4 values spanning two decades.
- **Gate width w** — the difficulty axis: sweep the door from clearly-passable to clearly-impassable through the critical width, with the analytic gate angle known throughout.
- **Scene family** — {volume-filled synthetic, **surface-only synthetic**, multi-door synthetic, Stonehenge, InteriorGS ×20, Replica ×5}.
- **κ, height band** — for real-data runs only.

### Baselines (all at *matched support count*, this is non-negotiable)
1. Uncompressed (the reference).
2. **Uniform random pruning.**
3. **Opacity / importance pruning** (LightGaussian-style).
4. **Voxel-grid clustering** at matched n.
5. **Octree/quadtree coarsening**, orientation via circumscribed-radius inflation.
6. **Clearance-adaptive quadtree** — leaf size proportional to local clearance. *This is the strongest baseline and the one most likely to match you.* If it does, say so.
7. **NanoGS moment-matching merge** (arXiv:2603.16103 **[S]**) — the closest geometric merge criterion in the literature.
8. **Geometry-only coarsening** (your existing control) — keep it, but stop calling it the comparison; it is a lower bound on baselines, not a competitor.
9. **ε-intrusion clause only** (topology clause disabled) — the ablation that tells you whether the topology gate earns its complexity.
10. **Topology clause only** (ε clause disabled) — the mirror ablation.
11. **Surface-only geometric filter alone** — promote your caveat to a first-class baseline.
12. **Surface-only + robot-conditioned** — *the missing cell*.

### Metrics

**Compression**
- Support count ratio; support *evaluations* per compile; bytes.

**Correctness — soundness (one-sided, currently your only claim)**
- Signed gate error, with an **explicitly stated sign convention and an explicit statement of which side of the sandwich it measures** (M_safe or M_possible). Report both sides separately. Never report one number called "gate error."
- Count of paths returned REACHABLE that **fail independent re-verification against the original supports**. **This must be exactly 0. It is your headline safety metric and you already have the machinery.**
- Certified clearance LB delta vs uncompressed: must be **≤ 0** for every query. (Your reported +3.7e-8 violates this — see §6.)

**Correctness — completeness preservation (the side you do not currently claim, and the one your title requires)**
- **Verdict transition matrix** over N queries: REACHABLE→{REACHABLE, UNKNOWN, UNREACHABLE}, and likewise from UNKNOWN and UNREACHABLE. The only forbidden transitions are →REACHABLE-that-fails-verification and REACHABLE→UNREACHABLE. **REACHABLE→UNKNOWN is the conservatism cost and must be reported as a rate, not hidden.**
- **Bottleneck distance** between the erosion-filtration persistence diagrams of the coreset and the uncompressed map, restricted to [b,a]. A number, not a flag.

**Efficiency**
- Coarsening time, compile time, per-query time — all in **absolute seconds** alongside ratios, on named hardware.
- **Scaling curves**: coarsening time and compile time vs n, log-log, with fitted exponents. *This is the single most important missing measurement for real data* (§5.5).
- **Amortization curve** — see §4.3.

### Statistics
- **≥100 randomized start/goal queries per scene** (matching Splat-Nav's standard), reported per scene, not pooled.
- Paired comparisons (same query, coreset vs uncompressed); bootstrap CIs on medians.
- **For every safety metric, report the worst case, not the mean.** A safety claim is a statement about the maximum, and a mean gate error is meaningless.
- **Replace "~40 arms" with a property-based randomized campaign.** Generate random scenes × robots × queries, N ≥ 10⁵. Zero violations in N trials gives, by the rule of three, a 95% upper confidence bound of 3/N on the violation rate — at N=10⁵ that is **< 3×10⁻⁵**, a citable number. Forty hand-chosen arms supports no such statement, and a reviewer will say so.
- Seed everything and publish seeds.

## 4.3 The amortization measurement you never made

Compile is once per (scene, robot); query is cheap. So the interesting curve is not over queries — it is over **robots and scenes**. Three curves:

- **C1 (per-scene, one robot):** total = coarsen + compile + Q·query, vs uncompressed compile + Q·query. From your numbers: coarsening 15–27 s, compile 10.5 s vs 153.07 s → coreset wins at Q=1 already, with net speedup **153.07/(10.5+15) = 6.00×** to **153.07/(10.5+27) = 4.08×** **[derived]**. Note this is *not* 14.6× — the 14.6× excludes the coarsening you had to pay for.
- **C2 (one scene, K robots), naive:** re-coarsen per robot → K·(coarsen) + K·(compile). Speedup degrades toward 4–6× and never improves.
- **C3 (one scene, K robots), exploiting §2.3 monotonicity:** coarsen **once** at r_max, compile K times → coarsen + K·compile vs K·(uncompressed compile). As K grows this tends to the full **14.6×**. **C3 is the plot that justifies the whole layer, and it is the direct payoff of stating the monotonicity theorem.** Draw it.

## 4.4 Falsifiable predictions

- **P1 (the decisive one).** On the **surface-only** synthetic scene, robot-conditioned coarsening reaches ≤ 40 supports at zero gate error — i.e. ≥ 3× beyond the 124 that the free geometric filter achieves. **REFUTED if it cannot get below ~90 supports** (i.e. < 1.4× beyond free), in which case the compression contribution is essentially the surface filter and the paper must be rewritten around the certificate rather than the ratio.
- **P2 (monotonicity).** A coreset built with probe radii covering [b, a_max] is admissible for every robot with a ≤ a_max. **REFUTED by a single counterexample** — which would mean the admissibility criterion is not actually monotone in r, i.e. the ε-intrusion clause can admit a merge that damages topology at a radius it never probes. *That is a plausible failure mode of the disjunction and you should hunt for it deliberately.*
- **P3 (partial vs total order).** For an incomparable robot pair (eccentric-small, round-large), neither coreset is adequate for the other. **REFUTED if one always subsumes**, which would collapse "robot-conditioned" to a single scalar and materially weaken the claim.
- **P4 (real data).** On ≥3 InteriorGS scenes at ≥10⁵ splats, compression at **zero verdict change** is ≥ 3× over the best matched-budget baseline. **REFUTED if < 1.5×.**
- **P5 (soundness).** Zero unsound verdicts in ≥10⁵ randomized trials. **REFUTED by one.**
- **P6 (scaling).** Compile is O(n) in supports; coarsening is O(n log n) or better. **REFUTED if coarsening is superlinear enough that at n=10⁵ it exceeds the compile time it saves** — which, at 15–27 s for n=484, is a live risk: naive pairwise hierarchy construction is O(n²), and O(n²) at n=10⁵ is ~4×10⁴× the work, i.e. **days**. Measure this before anything else on the real-data path.

## 4.5 What would refute the central claim

The claim is *preservation of certified mobility topology*. It is two-sided, so there are two refutations:

- **Soundness refutation:** any query where the coreset compiler answers REACHABLE and the returned path fails independent re-verification against the original supports. One instance kills the word "certified."
- **Completeness refutation (the one you are currently exposed to):** any query where the uncompressed compiler answers REACHABLE and the coreset answers UNREACHABLE. This is *precisely what your geometry-only control does* at n=28 (gate error −0.346) and n=12–15 (door sealed, −0.510). Your claim is that robot conditioning prevents it. **You have verified that on one scene, one robot, one door width.** A single (scene, robot, gate-width) triple where the robot-conditioned coreset seals a traversable gate refutes the headline. Hunt for it adversarially — sweep the gate width through the critical value at fine resolution, because that is where a preservation criterion tuned by a tolerance will break first.

---

# 5. REAL-DATA PLAN — 3DGS PLY → sound `GaussianSupport2D`

This is the actual blocker, and it is more subtle than it looks, because **the only truly sound reduction is the vertical shadow, and the vertical shadow requires floor/ceiling removal to be useful, and floor/ceiling removal is not sound.** The plan below makes every unsound step explicit, parameterized, and measured, rather than hiding it.

## 5.1 Input parsing

A 3DGS PLY carries per primitive: `x,y,z`; `scale_0..2` (**stored as log-scale in the reference implementation — apply `exp`**); `rot_0..3` (quaternion `w,x,y,z`, **stored unnormalized — normalize**); `opacity` (**stored as a logit — apply the sigmoid**); `f_dc_*`, `f_rest_*` (SH). Then

```
R  = quat_to_rotmat(normalize(rot))
S  = diag(exp(scale))
Σ  = R S S^T R^T          # 3×3, PSD
```

**Sanity gate before anything else:** verify metric scale and the up-axis. InteriorGS is XYZ = (Right, Back, Up) in metres; most other pipelines are Y-up or arbitrary-scale-from-SfM. **A 3DGS from monocular SfM has no metric scale at all, which makes robot dimensions meaningless.** Refuse to run on any scene without a metric-scale certificate (dataset-provided, or a measured known baseline). This is a hard precondition, not a warning.

## 5.2 Height band vs projection — the core soundness argument

A ground robot at pose (x, y, θ) occupies a **vertical prism** `P × [z_lo, z_hi]` where `P` is its 2D footprint. It collides iff the prism meets any obstacle. Therefore:

> A 2D obstacle set is sound iff, for every 3D obstacle primitive, its 2D proxy contains the **vertical projection of that primitive's intersection with the band [z_lo, z_hi]**.

This immediately rules out the common shortcut. **Slicing at a single height z₀ is unsound** — a table top at z=0.75 with a robot of height 1.0 is missed by a slice at z=0.10. (Note this is exactly what NavGSim appears to do with fixed Z-plane slices **[S — verify]**, and it is the flaw to point at.) Slicing at multiple heights and unioning is sound *only in the limit*; at finite spacing Δz it misses primitives thinner than Δz, which for 3DGS — where many splats are extremely flat — is a systematic, not a rare, failure.

**The sound and cheap answer is the shadow.** For the κ-level ellipsoid
`E_i(κ) = { p : (p−μ)ᵀ Σ⁻¹ (p−μ) ≤ κ² }`,
the **vertical projection onto the xy-plane is exactly the ellipse governed by the marginal 2×2 block of Σ**:

```
shadow(E_i(κ)) = { q ∈ R² : (q − μ_xy)ᵀ Σ_xy⁻¹ (q − μ_xy) ≤ κ² },   Σ_xy = Σ[0:2, 0:2]
```

(The shadow of an ellipsoid is governed by the *marginal* covariance — the block of Σ, **not** the inverse of the block of Σ⁻¹, which would give the conditional/slice. **Getting this backwards is the single most likely soundness bug in the whole importer; it silently shrinks every obstacle.** Write a unit test that checks `shadow ⊇ slice` at 20 random heights for 1000 random ellipsoids.)

In your support-function form, with `h(u) = u·μ + k √(uᵀ S u)`, this is:
```
μ_2D = μ_xy ;  S_2D = Σ_xy ;  k = κ
```
Exact, closed form, one line, and it contains every slice at every height. **Ship this first.**

**Optional tightening (band-restricted shadow), only if the full shadow proves too conservative.** Project `E_i(κ) ∩ {z_lo ≤ z ≤ z_hi}` instead. The slice at height z is an ellipse with
```
c(z) = μ_xy + Σ_xz Σ_zz⁻¹ (z − μ_z)
M(z) = (κ² − (z−μ_z)²/Σ_zz) · (Σ_xy − Σ_xz Σ_zz⁻¹ Σ_zx)
```
The union over z ∈ [z_lo, z_hi] is not an ellipse, but its support function is
`h_band(u) = max_{z∈[z_lo,z_hi]} [ u·c(z) + √(uᵀ M(z) u) ]`,
a 1-D maximization of a concave-ish function, solvable in closed form or by ternary search to any tolerance. **Then bound the result by an enclosing ellipse using the machinery you already built** — MVEE over sampled directions, minus a Lipschitz half-step, plus radial inflation, with a bounding-disc fallback. This is a pleasing structural point worth making in the paper: *the 3D→2D importer is an instance of the same certified outer-approximation procedure as the coreset's macro construction, applied to a one-parameter family instead of a finite set.* Same code, same proof obligation, same fallback.

## 5.3 Choosing κ — and being honest that it is an assumption, not a theorem

A Gaussian has unbounded support, so **no finite κ yields a genuinely sound obstacle.** Any claim otherwise is false, and a reviewer will find it. Worse, 3DGS scale parameters are fitted to minimize *rendering* loss, so a splat's extent is a rasterization footprint, not a physical occupancy extent — the relationship between "the surface" and "the κ-ellipsoid" is empirical, not derived.

**Therefore: state κ as a modelling assumption, then calibrate and inflate.**

- **Assumption (A1), stated explicitly in the paper:** *the physical obstacle surface is contained in the union of κ-ellipsoids of the splat map, inflated by δ.*
- **Calibration:** on a dataset with ground-truth geometry (Replica, ScanNet++), measure the **one-sided Hausdorff distance from the GT surface to the κ-ellipsoid union**, over a grid of κ ∈ {1, 1.5, 2, 2.5, 3}. Report the distribution: median, 95th percentile, **and max**.
- **Inflation:** set δ ≥ the measured max (or the 99.9th percentile, stated as such). Because your support function is `u·μ + k√(uᵀSu)`, **inflation by δ is literally `h ← h + δ`** — adding a disc of radius δ. Free to implement, trivially sound relative to A1, and it converts an empirical calibration into an auditable safety margin.
- **Report every headline result as a function of κ.** A single hidden κ is the kind of buried parameter that makes reviewers assume the rest is buried too.

## 5.4 Opacity, floors, ceilings, floaters — the unsound steps, made explicit

**Opacity.** Your standing rule (opacity does not define physical collision) is defensible and I would keep it — but note SPLANNING (arXiv:2409.16915) derives collision probability *from* the rendering model, so you owe a paragraph of justification, not a footnote. The practical problem: ignoring opacity entirely means every floater becomes an obstacle, and a scene full of floaters over the free space will drive everything to UNKNOWN — sound but useless.

**Resolution: two map layers, both reported.**
- **M_strict** — drop nothing except primitives provably outside the band. Sound relative to A1. **All safety claims are made on M_strict.**
- **M_denoised** — additionally drop primitives that are *geometrically unsupported*: fewer than N neighbours within radius d in the splat cloud (a density-outlier test on geometry, not on alpha). Opacity may be used **only to rank candidates for this test, never as the criterion itself** — that keeps your rule intact while remaining practical. **Unsound; label it so.**
- **Report the delta**: how many primitives, and how much free space, separate M_strict from M_denoised. If the delta is small, M_strict is what you ship and the whole issue evaporates. If it is large, you have discovered something worth reporting.

**Floor and ceiling removal.** Necessary — a ceiling splat's vertical shadow blocks the floor beneath it, so without removal the entire map is occupied. Procedure:
1. Estimate the floor plane (RANSAC on splat centres with near-vertical principal axis; or take it from the dataset's floorplan JSON where available — **InteriorGS gives it to you**).
2. Set `z_lo = floor + ground_clearance`, `z_hi = floor + robot_height`.
3. Drop primitive i iff its κ-ellipsoid is **entirely** outside the band. The exact test uses the support function in the z direction:
   - entirely above: `μ_z − κ√Σ_zz > z_hi`
   - entirely below: `μ_z + κ√Σ_zz < z_lo`
   Both are exact for the κ-ellipsoid, so **this step is sound relative to A1** — unlike everything else in this subsection. Keep every partially-overlapping primitive.
4. **Multi-floor buildings break this.** HM3D is multi-floor. Segment floors first (histogram of splat-centre z, or the dataset's floor labels) and run the importer once per floor, or refuse the scene.

## 5.5 Scale — the thing most likely to kill this

484 supports → 15–27 s of coarsening. Real scenes are 10⁵–10⁷ splats: **two to four orders of magnitude.** If hierarchy construction is O(n²), n=10⁵ is ~4×10⁴× the work — days per scene. If it is O(n log n) with the same constant, it is ~5×10³ s ≈ 1.4 h per scene, which is tolerable for a compile-once pipeline.

**You do not currently know which.** Measure the exponent before you write any importer code — subsample one Stonehenge PLY to n ∈ {500, 1K, 2K, 5K, 10K, 20K}, fit log-log. If the exponent is ≥ 1.8, the coarsening needs spatial hashing / BVH before real data is reachable at all, and that is a different (larger) engineering project than the importer.

Mitigations if it is superlinear: (i) grid/BVH-bucket the splats and coarsen per bucket, merging across bucket boundaries only in a second pass; (ii) apply the band filter and the surface filter *first* (they are cheap and, per your own caveat, the band filter alone may remove most primitives); (iii) exploit §2.3 — coarsen once at r_max, never per robot.

## 5.6 Pseudocode

```python
def import_3dgs_to_supports2d(ply_path, floor_z, robot_h, ground_clear,
                              kappa, delta, band_tight=False):
    """3DGS PLY -> list[GaussianSupport2D].  Sound relative to assumption A1:
       the physical obstacle is contained in the union of kappa-ellipsoids
       inflated by delta.  A1 is an ASSUMPTION, calibrated in calibrate_kappa()."""
    g = read_ply(ply_path)
    assert_metric_scale(g)                       # HARD precondition: no metric scale -> refuse
    mu    = g.xyz                                # (N,3)
    scale = np.exp(g.scale)                      # PLY stores log-scale
    R     = quat_to_rotmat(normalize(g.rot))     # PLY stores unnormalized quats
    Sigma = R @ (scale[...,None]**2 * np.eye(3)) @ R.transpose(0,2,1)   # (N,3,3)

    z_lo, z_hi = floor_z + ground_clear, floor_z + robot_h
    sd_z  = np.sqrt(Sigma[:,2,2])
    # exact support-function test in +-z: sound, no primitive that touches the band is dropped
    keep  = ~((mu[:,2] - kappa*sd_z > z_hi) | (mu[:,2] + kappa*sd_z < z_lo))
    mu, Sigma = mu[keep], Sigma[keep]

    supports = []
    for m, S in zip(mu, Sigma):
        if not band_tight:
            # full vertical shadow: marginal 2x2 BLOCK of Sigma (NOT inv of block of inv)
            mu2, S2, k = m[:2], S[:2,:2], kappa
        else:
            # band-restricted shadow: max over z of the slice support, then certified
            # enclosing ellipse via the SAME MVEE + Lipschitz-half-step + radial-inflation
            # + bounding-disc-fallback machinery used for the coreset macros.
            mu2, S2, k = certified_enclosing_ellipse(
                lambda u: max_z_slice_support(u, m, S, kappa, z_lo, z_hi))
        supports.append(GaussianSupport2D(mu=mu2, S=S2, k=k, inflate=delta))
    return supports
    # h(u) = u.mu2 + k*sqrt(u' S2 u) + delta      # inflation is just an added disc


def calibrate_kappa(ply_path, gt_mesh, kappas=(1,1.5,2,2.5,3)):
    """One-sided Hausdorff from GT surface -> kappa-ellipsoid union.
       Returns, per kappa: median / p95 / MAX uncovered distance.
       Choose delta >= MAX (or a stated high percentile). Report the whole table."""
    ...
```

**Unit tests that must exist before any result is quoted:**
1. `shadow ⊇ slice` at 20 random heights, 1000 random ellipsoids — catches the marginal-vs-conditional inversion.
2. Band filter never drops a primitive whose κ-ellipsoid intersects the band — 10⁴ randomized cases.
3. `h(u)` with inflation ≥ `h(u)` without, for all u.
4. Round-trip: a synthetic 3D scene with known 2D ground truth (a box wall) imports to a support set containing it.

## 5.7 Failure modes

| # | Failure | Consequence | Detection / mitigation |
|---|---|---|---|
| 1 | **Under-reconstructed thin structures** (chair legs, table legs, wires, railings) absent from the splat map | **Real obstacle simply does not exist in your input.** No conservative post-processing can recover it. | *Unfixable in principle.* Say so explicitly: **soundness is relative to the splat map, never to the physical world.** Measure it against GT meshes (Replica/ScanNet++) and report the miss rate as a first-class number. **This is the reviewer's kill shot on the word "certified" and you must disarm it yourself.** |
| 2 | Marginal/conditional inversion in the shadow | Silently *shrinks* every obstacle → unsound everywhere, no visible symptom | Unit test 1 above. |
| 3 | Floor mis-estimated | Whole map occupied (floor shadows everything) or floor removed leaving phantom free space over a stairwell | Cross-check against dataset occupancy map (InteriorGS provides one); assert the free-space fraction is within a plausible band. |
| 4 | **Multi-floor scene** (HM3D) | Upper floor shadows lower floor → map fully sealed | Detect via z-histogram multimodality; segment per floor or refuse the scene. |
| 5 | **Glass and mirrors** | 3DGS reconstructs reflected geometry as real → phantom obstacles behind mirrors and phantom free space through glass. *Phantom free space through glass is a genuine safety failure, not just conservatism.* | Replica annotates mirrors and glass — **use it to measure the effect rather than assert it is small.** |
| 6 | **View-ray needle splats / streaks** | Extremely anisotropic primitives cast huge shadows → over-conservative sealing of real passages | Report the distribution of condition numbers of Σ_xy; consider capping aspect ratio *and reporting how many primitives were capped*. |
| 7 | **No metric scale** (monocular SfM) | Robot dimensions meaningless; every result is arbitrary | Hard precondition in the importer; refuse. |
| 8 | Floaters | Sound but useless map, everything UNKNOWN | M_strict / M_denoised two-layer reporting (§5.4). |
| 9 | **Coarsening superlinear in n** | Pipeline never reaches real scale | Measure the exponent first (§5.5). |
| 10 | κ chosen by taste | Every downstream number is a free parameter in disguise | `calibrate_kappa()` against GT mesh; sweep κ in every table. |

## 5.8 SLURM plan (NYU HPC, sbatch, 62 GB overlay `xgrid_indoor_1_k2_full_64g.ext3`)

I have no access to the cluster and cannot verify module names, partition names, or the overlay's contents. **Treat this as a template with named TODOs, not a runnable script.**

**Step 0 — inventory (interactive, 30 min).**
```bash
srun --pty --mem=16G --time=1:00:00 /bin/bash
singularity exec --overlay /scratch/wg2381/xgrid_indoor_1_k2_full_64g.ext3:ro \
    <TODO: the .sif you normally use> /bin/bash
# inside: find the PLY, and record ground truth about the input
ls -la /ext3/  ; find / -name '*.ply' -size +10M 2>/dev/null | head
python -c "from plyfile import PlyData; d=PlyData.read('<PLY>'); \
           print(len(d['vertex']), d['vertex'].data.dtype.names)"
```
Record: **splat count, property names (confirm `scale_*` are log and `rot_*` unnormalized), coordinate bounds, up-axis, and whether the scene is metric.** Everything downstream depends on these five facts.

**Step 1 — mount discipline.** Mount the overlay **`:ro`** for all read jobs. Never write into a 62 GB overlay from an array job — concurrent writes to one ext3 overlay corrupt it. Write outputs to `/scratch/wg2381/splathjb/out/$SLURM_JOB_ID/`.

**Step 2 — scaling probe (do this before the importer).** Subsample the PLY to n ∈ {500, 1K, 2K, 5K, 10K, 20K}, run coarsening on each, fit the log-log exponent. **One CPU job, ~1 h, and it decides whether the real-data path is viable at all.**

**Step 3 — importer job (CPU, no GPU needed).**
```bash
#!/bin/bash
#SBATCH --job-name=gmc_import
#SBATCH --nodes=1 --ntasks=1 --cpus-per-task=8
#SBATCH --mem=64G --time=04:00:00
#SBATCH --output=/scratch/wg2381/splathjb/logs/%x_%j.out
#SBATCH --error=/scratch/wg2381/splathjb/logs/%x_%j.err
# NOTE: 64G is a guess sized to ~1e6 splats x (3+9+1) float64 plus working copies.
#       Re-derive from the Step-0 splat count before trusting it.
module purge                                    # TODO: cluster-specific
OUT=/scratch/wg2381/splathjb/out/$SLURM_JOB_ID ; mkdir -p $OUT
singularity exec \
  --overlay /scratch/wg2381/xgrid_indoor_1_k2_full_64g.ext3:ro \
  <TODO .sif> \
  /bin/bash -c "source /ext3/env.sh; \
    python -m gmc.import3dgs --ply <PLY> --floor-z <TODO> --robot-h 1.0 \
      --ground-clear 0.05 --kappa \$KAPPA --delta \$DELTA \
      --out $OUT/supports_k\${KAPPA}.npz"
```

**Step 4 — κ/δ sweep as a job array.** `--array=0-4` over κ ∈ {1,1.5,2,2.5,3}, each writing its own `supports_k*.npz`. Independent, embarrassingly parallel, cheap.

**Step 5 — κ calibration against GT.** Only on Replica/ScanNet++ (InteriorGS's occupancy PNG is a weaker but usable proxy). Emits the κ → (median, p95, max) uncovered-distance table that sets δ.

**Step 6 — coarsening + compile.** Separate job (this is the one that may need hours or a big-memory node). **Checkpoint after coarsening**, so a compile crash does not cost the coarsening. Persist the coreset to disk — that artifact *is* the compile-once product and its reuse across robots is the §2.3/§4.3-C3 result.

**Step 7 — query campaign.** ≥100 randomized start/goal pairs as an array job over the persisted coreset; every REACHABLE path re-verified against the **original** supports (not the imported ones, not the macros). Log the full verdict transition matrix.

**Practical cautions:** request the full walltime you need — a 4 h job killed at 3 h 55 m loses everything without Step 6's checkpoint; keep the overlay read-only in every array task; and stage the PLY to node-local scratch if the cluster has it, since 10⁶-splat PLY reads over a shared filesystem from 5 concurrent array tasks will thrash.

---

# 6. CRITICAL REVIEW — the attacks, and what blunts each

### A. The two internal inconsistencies (raise these before a reviewer does)

**A1. The gate-error sign contradicts the safety invariant.** The headline is "+2e-5"; the safety claim is "~40 arms, every gate error ≤ 0 (never optimistic)"; the geometry-only control is −0.346 (conservative) and −0.510 (sealed). Under the convention the control implies — **negative = conservative, positive = optimistic** — the headline result is 2×10⁻⁵ **on the unsound side**, contradicting the invariant.
Three readings: (i) the two statements use different sign conventions; (ii) the headline measures M_possible (the outer bound, where positive is *correct and expected*) while the invariant is about M_safe, and the report conflates the two sides of the sandwich; (iii) the headline is genuinely 2e-5 optimistic.
Reading (ii) is the most likely and the most benign — but **the reported results as written do not say which side of the sandwich any error refers to, and that ambiguity is fatal in review for a paper whose entire contribution is a two-sided certificate.**
*Blunting:* fix the convention, report M_safe and M_possible gate errors as two separate columns everywhere, and state the invariant per side (M_safe error ≤ 0, M_possible error ≥ 0). Then re-check all ~40 arms under the corrected convention. **Do this today; it may change what you believe about the method.**

**A2. The coreset certifies more clearance than the uncompressed map.** 0.021715945 vs 0.02171590806 — the coreset's LB is **larger by 3.7×10⁻⁸** (relative 1.7×10⁻⁶) **[derived]**. A coreset that only grows obstacles satisfies `dist(x, macros) ≤ dist(x, originals)`, so if both bounds come from the same estimator on the same path — and you report the **same** 72-node/74-edge graph, so it plausibly *is* the same path — the coreset's LB must be **≤** the uncompressed one. It is not.
Most likely benign explanation: float32/float64 or a different code path (radial inflation, bounding-disc fallback) between the two pipelines, at the 8th significant figure. Malignant explanation: a soundness leak in the macro construction.
*Blunting:* explain it in writing, in the paper. Until then, **do not describe the two results as "identical"** — they differ, in the direction that soundness forbids. If it is numerical, add an assertion `LB_coreset ≤ LB_uncompressed + tol` to the test suite and quote the tol.

### B. "Your compression number is an artifact of volume-filled walls"

**This is the strongest attack, and you found it yourself, which is to your credit — but you have not yet acted on it.** Real 3DGS primitives already lie on surfaces, so the interior splats your coarsener is compressing away do not exist in real data. Your own surface-only control gives **484→124 at zero gate error from a free geometric filter** — that is **3.90×** of the 17.29× total **[derived]**, obtained with no robot conditioning, no certificate, and no coreset.

The residual attributable to robot conditioning is therefore at most **124/28 = 4.43×** **[derived]** — *and even that is an assumption*, because the coreset was never run on the surface-only variant. It may reach 28. It may reach 60. It may reach 20.

*Blunting:* **run the missing cell** (surface-only × robot-conditioned), and then **report 4.4× (or whatever it is) as the headline**, with 17.3× demoted to a footnote about the synthetic scene's redundancy. A 4× certified reduction is a perfectly respectable result. A 17× result that a reviewer decomposes into 3.9× free + 4.4× yours, in public, at review time, is a rejected paper — because it reads as either carelessness or salesmanship, and reviewers cannot tell which.

### C. "One scene family, one robot for the headline"

484 disc supports, one door, one width (0.60 m), one robot (a=0.50, b=0.20), one query. The two-door scene exists but is used only for the adequacy matrix, with two robots.
The analytic gate half-angle 0.509740 rad — your *primary metric* — exists **only because the scene is trivial enough to solve in closed form.** On any real scene it evaporates, and you have no stated replacement.
*Blunting:* (i) sweep the gate width through the critical value, which turns one point into a curve and stresses the criterion exactly where it should break; (ii) a robot grid including incomparable pairs (§4.4-P3); (iii) ≥100 randomized queries per scene, matching Splat-Nav's standard; (iv) **name the replacement for the analytic gate on real scenes now**: independent path re-verification against original primitives, verdict agreement with the uncompressed compiler, certified clearance delta, and a high-resolution brute-force reference planner as pseudo-ground-truth.

### D. "Does 17× survive on surface-only real splats?"

On the evidence: **almost certainly not, and you should not need a reviewer to tell you.** Best case, per (B), ≈4.4× — and real splats add three effects your synthetic scene lacks: (i) massive anisotropy (streaks, needles) that makes enclosing ellipses loose; (ii) noise and floaters that fragment the hierarchy; (iii) 10⁵–10⁷ primitives, where your coarsening's unmeasured complexity may not even terminate in reasonable time (§5.5).
*Blunting:* the only thing that blunts this is running it. Predict a number in advance (P4: ≥3× at zero verdict change), then publish whatever you get. A pre-registered prediction that comes in low is a credible paper; a post-hoc number is not.

### E. "The 'same 72n/74e graph' is an artifact of a trivially simple scene"

72 nodes and 74 edges is **cyclomatic number 74 − 72 + 1 = 3** **[derived]** — a near-tree with three independent loops. A roadmap that simple cannot discriminate between a good coreset and a mediocre one: almost any conservative approximation that keeps the door open produces the same graph. **The graph-identity result is close to vacuous, and a reviewer will say so in one sentence.**
*Blunting:* either drop the claim, or replace it with something that has discriminative power — graph edit distance and **bottleneck distance between the erosion-filtration persistence diagrams**, on scenes with cyclomatic number in the tens (multi-room floorplans; InteriorGS scenes have exactly this structure). Graph *identity* on a scene with 3 loops is not evidence; graph *near-identity* on a scene with 40 loops is.

### F. "Compile-once/query-many amortization was never measured"

Correct, and worse: **the amortization you would naturally claim is not the one that matters.** Compile is per-(scene, robot) and query is cheap, so the interesting axis is robots and scenes, not queries.
Also: your reported 14.6× **excludes the 15–27 s of coarsening you had to pay to get it.** Including it, the honest single-shot end-to-end figure is **4.08×–6.00×** **[derived]**: 153.07/(10.5+27) = 4.08, 153.07/(10.5+15) = 6.00. **Report that number, and report 14.6× only as the marginal per-compile speedup, labelled as such.** A reviewer who works this out unaided will not be charitable about the difference.
*Blunting:* the §4.3 C1/C2/C3 curves, with C3 (coarsen once at r_max, compile K times, → 14.6× as K grows) as the justification for the whole layer. **The monotonicity theorem of §2.3 is what makes C3 legitimate** — without it, you must re-coarsen per robot and the speedup is stuck at 4–6× forever.

### G. "Your baseline is your own uncompressed implementation"

153.07 s to compile a **484-ellipse 2D scene** is slow in absolute terms. A reviewer will ask what the constant factor is and whether the baseline is competently implemented — because a 14.6× speedup over a slow baseline may be a 2× speedup over a good one, or a slowdown.
*Blunting:* **run HRM's released C++ (github.com/ChirikjianLab/hrm) on its own SE(2) narrow-passage benchmarks and put the absolute seconds in the table.** It is public, it is the closest prior art, and it is the paper you collided with. Not running it will be read as avoidance. Report absolute times on named hardware alongside every ratio.

### H. "'Certified' against what?"

Your certificate is relative to *your support-function model of a splat set*. It says nothing about whether the splats represent the world. A missing chair leg (§5.7-1) is invisible to every proof in the pipeline.
*Blunting:* say it first, in the abstract, in one sentence: *"soundness is relative to the input primitive set; we separately quantify the primitive set's fidelity to ground-truth geometry."* Then quantify it — GT-mesh coverage on Replica/ScanNet++ (§5.3). A paper that states its own limit precisely is trusted on everything else; a paper that lets a reviewer find the limit is trusted on nothing.

### I. "Forty arms is not a safety argument"

For a method whose selling point is a proof, "~40 arms, all conservative" is the wrong kind of evidence: it is a weak empirical claim standing where a theorem should be, and it supports no statement about the tail.
*Blunting:* prove the one-sided invariant (it should follow directly from the containment certificate), and replace the arms with a property-based randomized campaign at N ≥ 10⁵ (§4.2), reported with the rule-of-three bound. **Either the proof or the 10⁵ campaign is worth more than forty hand-picked cases; both together close the issue permanently.**

### J. "The topology clause may be redundant"

Per stability (Cohen-Steiner–Edelsbrunner–Harer 2007), an ε-intrusion bound already implies a bounded change in the persistence diagram. So the disjunction "ε-intrusion **OR** unchanged signature" may have a branch that never fires, or fires only in a narrow regime. A topology-literate reviewer will ask **how many merges the signature clause admits that the ε clause rejects.**
*Blunting:* instrument it and report the count (§4.2 ablations 9 and 10). If small: delete the clause, and the method gets simpler, faster, and easier to prove. If large: that is a headline you are currently not claiming — *"a persistence-based gate admits N× more merges than any metric-intrusion bound can."*

### K. "The claim is two-sided; the guarantee is one-sided"

"Provably preserves certified mobility topology" requires **both** no-false-REACHABLE (soundness — your ≤0 invariant, measured) **and** no-false-UNREACHABLE (completeness preservation — never claimed, never proved, only exhibited on one scene by contrast with the geometry-only control). **The gap between the title and the guarantee is the most likely single reason for a reject.**
*Blunting:* either prove the second direction under stated conditions, or retitle honestly ("conservative robot-conditioned coarsening with measured mobility preservation") and make the empirical completeness rate a first-class reported metric (the REACHABLE→UNKNOWN transition rate of §4.2).

### L. The scoop risk

SE(2) NavMesh (ETH RSL, July 2026) is your problem statement on hardware. SAFER-Splat (same group as Splat-Nav) says 10⁵ primitives need no reduction at all. FOCI (also ETH RSL) does orientation-aware narrow passages on real splats today. NanoGS (2026) does training-free geometric splat merging. **Every ingredient of your paper exists in someone's repo; only the combination and the certificate do not.** That is a real contribution and a short-lived one. It argues for RA-L (fast turnaround) over T-RO (long review, high scoop exposure).

---

# NEXT 5 CONCRETE ACTIONS, prioritized

**1. Resolve the two internal inconsistencies (§6-A). Half a day. Do this before anything else.**
Fix the gate-error sign convention; separate M_safe from M_possible in every reported error; re-check all ~40 arms under the corrected convention; explain the +3.7×10⁻⁸ clearance-LB inversion and add `assert LB_coreset <= LB_uncompressed + tol` to the test suite. **Until A1 is resolved you do not know whether your headline result is conservative or 2e-5 unsound, and every downstream decision rests on that.**

**2. Run the missing cell: robot-conditioned coarsening on the surface-only scene (§6-B, P1). One day.**
This is the cheapest decisive experiment you have. It converts the contaminated 17.3× into an honest number, and it is the single result that most changes whether this is a paper. **Predict the number before you run it**, then publish what you get. If it lands below ~1.4× beyond the free surface filter, stop and reposition around the certificate rather than the compression ratio.

**3. State and prove the monotonicity proposition; rebuild the amortization story on it (§2.3, §4.3-C3). One to two days.**
Write the three-line proof (sealing propagates upward in erosion radius ⟹ verification at r_max certifies all smaller robots). Replace the adequacy matrix with proposition + verification. Then measure the C3 curve — coarsen once at r_max, compile for K robots — which is what turns 4–6× into 14.6× and is **the strongest positive result you have.** Also hunt deliberately for a P2 counterexample: the ε-intrusion clause could admit a merge that damages topology at a radius it never probes, which would break the proposition and is worth knowing now.

**4. Read SE(2) NavMesh (arXiv:2607.01454) and HRM's released C++, and run HRM as an external baseline (§1d, §6-G). Three to five days.**
Two independent existential threats, both resolvable by reading. If NavMesh already chooses its polygon-simplification tolerance from footprint radii, the coreset layer must be repositioned immediately. And HRM's C++ on its own narrow-passage benchmarks, in absolute seconds, is the only thing that makes any timing claim credible — 153.07 s for 484 ellipses will otherwise be read as a slow baseline.

**5. Measure the coarsening scaling exponent, then build the importer, full-shadow-first (§5.5, §5.2). One to two weeks.**
Subsample `stonehenge.ply` (five minutes from the FOCI repo) to n ∈ {500 … 20K}, fit the log-log exponent for coarsening and compile. **If the exponent is ≥1.8, spatial-hashing the hierarchy is a prerequisite to real data and a bigger project than the importer — you need to know that before you write importer code, not after.** Then build the importer: full vertical shadow (marginal Σ_xy block, `k=κ`) plus the exact band filter, with the four unit tests of §5.6 — especially `shadow ⊇ slice`, which catches the silent marginal/conditional inversion. Ship band-tightening only if the full shadow proves too conservative. Target InteriorGS (native PLY, metric, floorplans with labelled doors, occupancy ground truth, no access friction) and validate κ against Replica's GT mesh.

---

## Appendix — derived arithmetic used above

| Quantity | Computation | Value |
|---|---|---|
| Total compression | 484 / 28 | 17.29× |
| Free surface-only filter | 484 / 124 | 3.90× |
| Residual attributable to robot conditioning (**assumes coreset also reaches 28 on surface-only — untested**) | 124 / 28 | 4.43× |
| Marginal compile speedup (as reported) | 153.07 / 10.5 | 14.58× |
| **End-to-end incl. coarsening, best case** | 153.07 / (10.5 + 15) | **6.00×** |
| **End-to-end incl. coarsening, worst case** | 153.07 / (10.5 + 27) | **4.08×** |
| Compile time saved per compile | 153.07 − 10.5 | 142.57 s |
| Safe-graph cyclomatic number | 74 − 72 + 1 | 3 |
| Clearance LB discrepancy (coreset **higher**) | 0.021715945 − 0.02171590806 | +3.7×10⁻⁸ (rel. 1.7×10⁻⁶) |
| Geometry-only recovered gate at n=28 | 0.509740 − 0.346 | ≈0.1637 rad (32% of analytic) |
| Geometry-only at n=12–15 | 0.509740 − 0.510 | ≈0 → door fully sealed |
| Rule-of-three bound, 0 violations in 10⁵ | 3 / 10⁵ | < 3×10⁻⁵ (95% upper) |

## Appendix — citation verification status

**[V] — confirmed:** Ruan et al. T-RO 39(1):110–127 2023 / arXiv:2104.04658 / github.com/ChirikjianLab/hrm (DOI suffix unverified); Splat-Nav arXiv:2403.02751 / TRO 2025 / 10.1109/TRO.2025.3552348; SAFER-Splat arXiv:2409.09868; SPLANNING arXiv:2409.16915; FOCI arXiv:2505.08510; GaussNav arXiv:2403.11625; 3DGS.zip arXiv:2407.09510 / CGF 2025; Agarwal–Har-Peled–Varadarajan MSRI 52:1–30 2005; Phillips arXiv:1601.00617 / HDCG 3rd ed.; Todd–Yıldırım DAM 155(13) 2007; Edelsbrunner–Letscher–Zomorodian DCG 28:511–533 2002; Cohen-Steiner–Edelsbrunner–Harer DCG 37:103–120 2007; Edelsbrunner–Kirkpatrick–Seidel IEEE T-IT 29(4):551–559 1983; Edelsbrunner–Mücke ACM TOG 13(1) 1994; OctoMap Auton. Robots 34(3) 2013 / 10.1007/s10514-012-9321-0; IRIS WAFR 2014 / STAR 107; GCS Science Robotics 2023; CDF arXiv:2406.01137 / RSS 2024; Selective Densification arXiv:2507.15710 + arXiv:2002.04941 + arXiv:1611.00111; SE(2) NavMesh arXiv:2607.01454 (existence, authors, abstract); Yang et al. arXiv:2411.05279; HM3D arXiv:2109.08238; Replica arXiv:1906.05797; ScanNet++ arXiv:2308.11417; InteriorGS (repo fetched); MP3D/Gibson licensing.

**[S] — snippet-level only, re-read before citing:** all Splat-Nav Table III splat counts; FOCI/SPLANNING/SAFER-Splat method internals; NanoGS arXiv:2603.16103; GaussianPOP arXiv:2602.06830; NavGSim arXiv:2603.15186; NVSim arXiv:2510.24335; Splatwizard arXiv:2512.24742; SUCCESS-GS arXiv:2512.07197; camera-agnostic pruning arXiv:2603.21933; LightGaussian / Mini-Splatting / RadSplat specifics; CDFlow arXiv:2509.13771; Neural C-space Barriers arXiv:2503.04929; task-driven map compression arXiv:2503.10843 / 2506.20579 / 2309.13451; Sparse 3D Topological Graphs arXiv:1803.04345; Topology-Preserving Terrain Simplification arXiv:1912.03032.

**Note on post-cutoff items:** everything dated after 2025-05 is outside my training data and was established from search-result metadata alone, with `arxiv.org` unreachable from this environment. The arXiv IDs and titles are reliable; **no claim about those papers' internals should reach a submission without the PDF open.**

