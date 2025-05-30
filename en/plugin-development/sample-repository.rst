==================
Sample Repository
==================

The source code for the sample plugins created in this guide is publicly available as a repository on Github. The address is https://github.com/choreonoid/plugin-dev-guide.git. We will call this repository the "sample repository". It contains the source code for all samples created in this guide, stored in separate directories corresponding to each serial number.

In this repository, all samples use the same plugin name "DevGuidePlugin". This is because if each sample becomes a separate plugin and they accumulate as you progress through the guide, conflicts would occur between plugins. For this reason, we use the same plugin name and ensure that only one sample is built and executed at a time.

The samples can be built in the Choreonoid build environment. In that case, clone the repository into the ext directory of Choreonoid. This will add the following item to Choreonoid's CMake configuration:

* DEV_GUIDE_SAMPLE_INDEX

Set the serial number of the sample you want to build and execute here. Then the sample with that serial number will be built and installed.

The serial numbers for each sample are shown below:

* 01: :doc:`minimum-sample`
* 02: :doc:`signal-sample`
* 03: :doc:`item-operation-sample`
* 04: :doc:`toolbar-sample`
* 05: :doc:`new-item-type-sample`
* 06: :doc:`item-scene-sample`
* 07: :doc:`item-property-sample`
* 08: :doc:`item-project-save-sample`
* 09: :doc:`item-file-io-sample`
* 10: :doc:`item-creation-io-customization-sample`
* 11: :doc:`create-view-sample`

The heading of each section introducing a sample includes "(S serial number)" notation to indicate the sample number.