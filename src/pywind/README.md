# Wind Generation

## Theory

The theory for generation a wind field is described in the following paper:

"Learning the structure of wind: A data-driven non-local turbulence model for
the atmospheric boundary layer" , B. Keith, U. Khristenko, B. Wohlmuth
Physics of Fluids 33 (9), 095110, 
[10.1063/5.0064394](https://doi.org/10.1063/5.0064394)

The paper describes various methods but we keep it very basic here. Basically, 
the generation of a wind field is divided into three steps.

1. Definition of a divergence free mean wind field. A trivial choice is a 
   constant vector.
2. Construction of divergence free turbulence field. 
3. wind = mean_wind + turbulent_wind

Step 1) and 3) are rather trivial while step 2 is not. The paper gives insights 
on multiple approaches but ultimately it boils down to  the equation (30). 
Hence, a divergence free synthetic turbulence field can be generated as follows.

1. Generate a discrete vector field in R^3 whose vectors (or rather their 
   components) are sampled from a Gaussian distribution.
2. Compute the Fourier transformation of the noise in all spacial directions.
3. Multiply the result point-wise with a kernel function. (There are many 
   kernels possible but we choose the simplest yielding a homogenous field, i.e.
   the Van Karman kernel.)
4. Compute the inverse Fourier transform of the result of the previous step - 
   again along all dimension.

This procedure has been implemented in a repository associated with the paper
and we build upon this here.

## Running the code

1. Download the repository of the paper by executing the download script
   ```bash
   ./download_repository.sh
   ```
2. Set up you python environment by installing the packages listed in 
   `required_pypackages.txt` and activating the environment. We tested it with
    python 3.9.6. For instance with conda:
   ```bash
   conda create -n wind $(cat required_pypackages.txt)
   conda activate wind
   ```
3. Execute `main.py` by running 
   ```bash
   python main.py
   ```
4. The results are saved in the folder `src/pywind/data/`. Open to ParaView, 
   navigate to `src/pywind/data/`, open `wind.vti` (via `ParaView/File/Open` 
   **not** via `ParaView/File/Load State`). Make the wind visible by clicking 
   the eye next to the `wind.vit` on the left side. Change the `Coloring` option
   from `Solid Color` to `wind`. In the box next to it, choose whatever you want
   to visualize, e.g. `x,y,z,Magnitude`. Lastly, choose the field 
   `representation` as `points` and you should be able to see the turbulent
   field. The same folder also contains the raw data as `wind.npy` for further 
   processing.