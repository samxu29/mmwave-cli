from setuptools import setup, Extension
from Cython.Build import cythonize

# Project root directory
ROOT_DIR = "ti"

# Directories containing headers and source files
MMWLINK_IDIR = f"{ROOT_DIR}/mmwavelink/src"
MMWLINK_H_IDIR = f"{ROOT_DIR}/mmwavelink/include"
MMWETH_IDIR = f"{ROOT_DIR}/ethernet/src"
MMWAVE_IDIR = f"{ROOT_DIR}/mmwave"
#CLI_OPT_IDIR = "opt"
#TOML_CONFIG_IDIR = "toml"

# All `.c` source files to compile
sources = [
    "mmwcas.pyx",         # Main Cython file
    f"{MMWLINK_IDIR}/rl_controller.c",
    f"{MMWLINK_IDIR}/rl_device.c",
    f"{MMWLINK_IDIR}/rl_driver.c",
    f"{MMWLINK_IDIR}/rl_monitoring.c",
    f"{MMWLINK_IDIR}/rl_sensor.c",
    f"{MMWETH_IDIR}/mmwl_port_ethernet.c",
    f"{MMWETH_IDIR}/mtime.c",
    f"{MMWAVE_IDIR}/crc_compute.c",
    f"{MMWAVE_IDIR}/mmwave.c",
    f"{MMWAVE_IDIR}/rls_osi.c",
#    f"{CLI_OPT_IDIR}/*.c",
#    f"{TOML_CONFIG_IDIR}/*.c"
]

# Build the extension module
extensions = Extension(
        name = "mmwcas",          # Output module name
        sources=sources,        # All source files
        include_dirs=[
#            ".",                # Current directory
            MMWLINK_IDIR,       # mmwlink directory
            MMWLINK_H_IDIR,     # mmwlink header directory
            MMWETH_IDIR,        # mmwethernet directory
            MMWAVE_IDIR,        # mmwave directory
#            CLI_OPT_IDIR,       # cliopt directory
#            TOML_CONFIG_IDIR    # tomlconfig directory
        ],
        extra_compile_args=["-w", "-Wno-error=incompatible-pointer-types", "-Wno-error=int-conversion"],  # extra compile flags (e.g. disable warnings)
        libraries=["pthread", "m"]  # link pthread and math libraries
    )

# Setup configuration
setup(
    name="mmwcas",
    ext_modules=cythonize(extensions),
)
