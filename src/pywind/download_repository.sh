#!/bin/bash
# -----------------------------------------------------------------------------
#
# Copyright (C) 2021 Some infos to come
#
# -----------------------------------------------------------------------------

# Download the wind generation repository from zenodo
wget https://zenodo.org/record/5076306/files/DRD_Wind.zip
# Remove __MACOSX from zip folder
zip -d DRD_Wind.zip "__MACOSX*"
# Unzip the files to DRD_Wind
unzip -u DRD_Wind.zip
# Delete downloaded folder
rm DRD_Wind.zip
