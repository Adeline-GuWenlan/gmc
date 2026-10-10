INVALID as a collision test (kept as evidence). Job 19506846. The synthetic wall's 54 Gaussians had the exact identity
quaternion [1,0,0,0]; SplatNav's `ellipsoids.covariance_utils.quaternion_to_rotation_matrix` returns NaN for any
quaternion with an exactly zero vector part, so their covariances were NaN and Splat-Plan never saw the wall (the
figure shows all three paths crossing x = 0; the clearance check returned NaN). The valid rerun is in `../splatnav/`.
