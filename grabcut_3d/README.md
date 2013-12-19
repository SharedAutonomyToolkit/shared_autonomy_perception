This package provides a segmentation library that is halfway between the standard RGB algorithm from openCV and the Grabcut3D version described in Ben Pitzer's ICRA2011 paper 

In this version, depth can only be integrated into the segmentation as one of the color channels, using a GMM. The separate histogram approach from the paper is not implemented, but would be easy to do so.