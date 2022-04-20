# Propagation-Loss-Models
Comparison of empirical propagation path loss models with NS-3 simulator. Based on the paper ["Comparison of Empirical Propagation Path Loss Models for Mobile Communication" by Michael Mollel and Dr. Michael Kisangiri](https://www.researchgate.net/publication/266386690_Comparison_of_Empirical_Propagation_Path_Loss_Models_for_Mobile_Communication).

## How to use
1. Put files in the appropriate locations. Structure of the repo matches ns-3 directory structure so files can just be copied over.
2. Run `./waf build` to build the new models
3. Run empirical-propagation-loss-comparison.cc with Waf