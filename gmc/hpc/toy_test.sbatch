#!/bin/bash
#SBATCH -p compute
#SBATCH -q cair
#SBATCH -c 4
#SBATCH --mem=16G
#SBATCH -t 02:00:00
#SBATCH -J gmc_toy
#SBATCH -o /scratch/sy2366/Project/gmc/outputs/slurm-%j.out

set -x
SING=/share/apps/NYUAD5/singularity/4.2.0/bin/singularity
PROJ=/scratch/sy2366/Project/gmc
mkdir -p $PROJ/outputs
SIF=/share/apps/admin/singularity-images/centos-8.2.2004.sif
OVL=$PROJ/gmc_env.ext3
if [ ! -f $OVL ]; then
  $SING overlay create --size 3072 $OVL || exit 3
fi
$SING exec --overlay $OVL $SIF bash -c '
set -e
PY=/ext3/miniconda3/envs/gmc/bin/python
if [ ! -x /ext3/miniconda3/bin/conda ]; then
  wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/mc.sh
  bash /tmp/mc.sh -b -p /ext3/miniconda3
fi
# conda 26 may register named envs outside the overlay (bound $HOME);
# force the env INTO the overlay with an explicit prefix, pip included
if [ ! -x $PY ]; then
  rm -rf /ext3/miniconda3/envs/gmc
  /ext3/miniconda3/bin/conda create -y -p /ext3/miniconda3/envs/gmc python=3.12 pip
fi
$PY -m pip install -q "numpy>=2" scipy shapely networkx pyyaml matplotlib pytest
cd /scratch/sy2366/Project/gmc
export PYTHONPATH=src
export MPLBACKEND=Agg
echo "=== VERSIONS ==="
$PY -c "import sys, numpy, scipy, shapely, networkx; print(sys.version.split()[0], numpy.__version__, scipy.__version__, shapely.__version__, networkx.__version__)"
echo "=== PYTEST FULL SUITE ==="
$PY -m pytest -q
echo "=== CLI TOY DEMO ==="
$PY -m gmc.cli synth single-door --width 0.6 --robot "a=0.50 b=0.20" -o scenes/synthetic/door_hpc.yaml
RUN="outputs/door_run_${SLURM_JOB_ID}"
$PY -m gmc.cli compile scenes/synthetic/door_hpc.yaml --config configs/toy.yaml -o "$RUN"
$PY -m gmc.cli query "$RUN" --start "-2,0,0.3" --goal "2,0,0.3"
PLAN=$(find "$RUN/queries" -type f -name path.json | sort | head -1)
test -n "$PLAN"
$PY -m gmc.cli verify "$PLAN"
echo "=== GMC_TOY_TEST_DONE ==="
'
