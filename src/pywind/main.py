################################################################################
#
# This part of the code base is based on and patially taken from the source code
# of the following paper:
#   "Learning the structure of wind: A data-driven nonlocal turbulence model for
#   the atmospheric boundary layer" , B. Keith, U. Khristenko, B. Wohlmuth
#   Physics of Fluids 33 (9), 095110, https://doi.org/10.1063/5.0064394
# The original (and way more exhaustive) source code can be found here:
#   https://doi.org/10.5281/zenodo.5076306
#
# Here, we only use the core functionality and keep it rather basic.
#
# Authored by: Tobias Duswald, Ustim Khristenko
#
################################################################################

import os
import numpy as np
from DRD_Wind.source.WindGeneration.CovarianceKernels import (
    VonKarmanCovariance,
)
from DRD_Wind.source.WindGeneration.GaussianRandomField import (
    VectorGaussianRandomField,
)


def CheckWorkingDir():
    """
    Checks if we're in the subfolder '/pywind'. If not, it raises an error.
    """
    if os.getcwd().split("/")[-1] != "pywind":
        print(
            "We're in the wrong folder. Please make sure that we are in",
            "src/pywind.",
        )
        raise OSError()


def CreateDataDir():
    """
    Creates a directory called '/data' if it is not there yet.
    """
    CheckWorkingDir()
    if not os.path.isdir("data"):
        os.mkdir("data")


def SaveToParaview(wind_field, grid_dimensions, grid_levels):
    """
    Exports the wind_field to the paraview file format '.vti'.
    """
    from pyevtk.hl import imageToVTK

    CreateDataDir()
    file_name = "data/wind"
    spacing = tuple(grid_dimensions / (2.0 ** grid_levels + 1))
    wind_field_vtk = tuple(
        [np.copy(wind_field[..., i], order="C") for i in range(3)]
    )
    cell_data = {
        "grid": np.zeros_like(wind_field[..., 0]),
        "wind": wind_field_vtk,
    }
    imageToVTK(file_name, cellData=cell_data, spacing=spacing)


def SaveToDisk(wind_field):
    """
    Exports the wind_field to data/wind.npy
    """
    CreateDataDir()
    np.save("data/wind", wind_field)


def Main():
    """
    1) Generate Gaussian noise X
    2) Compute Convolution as F^(-1)[G[F[X]]]
    3) Export results to .npy and .vti
    """
    print("Start wind generation ..")

    # --------------------------------------------------------------------------
    #   PARAMETERS FOR WIND FIELD
    # --------------------------------------------------------------------------

    # Parameters extracted from original repository's script
    # (script_GernerateWind) with method "VK" and setting a pdb trace in line
    # 103.
    friction_velocity = 2.683479938442173
    reference_height = 180
    E0 = 3.2 * friction_velocity ** 2 * reference_height ** (-2 / 3)
    L = 0.59 * reference_height

    # Parameters to steer the the size of the output shape (not 100% clear yet)
    noise_shape = np.array([490, 490, 490, 3])
    grid_levels = np.array([7, 7, 7])

    # Define the Covariance-Kernel. There are different kernels available but
    # VonKarman is simplest kernel generating a homogeneous wind profile.
    print("Initialize covariance kernel ..")
    Covariance = VonKarmanCovariance(ndim=3, length_scale=L, E0=E0)

    # --------------------------------------------------------------------------
    #   GENERATE TURBULENT WIND FIELD
    # --------------------------------------------------------------------------

    # Initilize the RandomField. We choose a 3D vector field with Gaussian noise
    # to generate the turbulent flow.
    print("Initialize vector field ..")
    RF = VectorGaussianRandomField(
        grid_level=grid_levels,
        grid_shape=noise_shape[:-1],
        ndim=3,
        sampling_method="vf_fftw",
        Covariance=Covariance,
    )

    # We first generate the noise, e.g a random vector field whose entries are
    # determined via a gaussian random process.
    print("Generating random field ..")
    noise = RF.sample_noise(noise_shape)

    # In the paper (see top), it is explained that one can compute a convolution
    # of a kernel and the random field to obtain a divergence free turbulance
    # model that one can add to another arbitrary (divergence free) mean wind
    # field. A trivial example for such a mean wind field is a constant vector.
    # The following code line basically evaluates equation (30) of the
    # respective paper: wind = FFT^(-1)[Ker * FFT[noise]], where '*' is a
    # component wise multiplication.
    print("Computing convolution ..")
    wind = RF.sample(noise)

    # --------------------------------------------------------------------------
    #   EXPORT WIND FIELD TO DISK
    # --------------------------------------------------------------------------

    # Todo(tobias,moritz): come up with interface to C++, i.e. suitable fromat
    # for I/O. For the time beeing, export to ParaView and .npy format.
    print("Saving results ..")
    SaveToParaview(wind, noise_shape[:-1].astype(float), grid_levels)
    SaveToDisk(wind)
    print("Finished.")


if __name__ == "__main__":
    Main()
