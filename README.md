# Control-Simulation-Demo
The repo is meant to pair with a lecture in using Python for Control and Simulation in potential manufacturing applications. The code and repository will leverage open-source libraries available through pip and conda forge.

## Notebook
The `verify/verify_environment.ipynb` notebook checks that required libraries import cleanly and includes an interactive mass-spring-damper demo. Use the `m`, `c`, and `k` sliders to see how parameter changes affect the system response.

## Requirements
If you run the notebook outside a fully featured environment, ensure `ipywidgets` is available for the sliders.

## Install
1. Create and activate a Python 3.10+ environment.
2. Install dependencies with pip:
   - `python -m pip install control numpy plotly ipywidgets`
3. Launch Jupyter and open `verify/verify_environment.ipynb`.

If you prefer conda:
1. `conda create -n control-demo python=3.10`
2. `conda activate control-demo`
3. `conda install -c conda-forge control numpy plotly ipywidgets`
