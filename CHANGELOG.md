# Changelog for package openpi runtime node

tros_0.1.0 (2025-12-13)
------------------
1. Add openpi runtime package based on pi0 model infer server on RDK S600 platform.
2. Add Subscription of images and robotic arm's joint state.
3. Get pi0 model infer result.

tros_0.1.1 (2025-12-18)
------------------
1. Add piper config.

tros_0.1.2 (2025-12-19)
------------------
1. Add scripts for piper.

tros_0.1.3 (2025-12-19)
------------------
1. Modified /embodiments/piper/collect_data.py

tros_0.1.4 (2026-1-5)
------------------
1. Add nodes for data collection
2. Add bag_2_hdf5.py and rename_hdf5.py for data collection
3. Modified README

tros_0.2.0 (2026-1-29)
------------------
1. Add .hbm model infer server


tros_0.2.1 (2026-2-2)
------------------
1. Organized the script files under the openpi_runtime/scripts directory
2. Add the retrieval path for the model file under the openpi_runtime/resource directory


tros_0.2.2 (2026-2-3)
------------------
1. Improved the pipeline for RDKs600 to deploy pi0
2. Optimized the action execution phase, using a first-order low-pass filter + linear interpolation algorithm to make the motion trajectory smoother
3. Updated the acquisition methods for models and datasets, as well as data collection documents and readme documents
