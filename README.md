
![Overview](/Figures\/Overview.PNG)
![Results](/Figures\/Results.PNG)

# Partial-Correspondence-3D-NNF
This repository contains the implementation of the work ["Partial Correspondence of 3D Shapes using Properties of the Nearest-Neighbor Field"](https://eur01.safelinks.protection.outlook.com/?url=https%3A%2F%2Fdoi.org%2F10.1016%2Fj.cag.2019.05.011&amp;data=02%7C01%7Cnyarbel%40campus.technion.ac.il%7Cb2aae53e68194f76676708d6e3aa10c8%7Cf1502c4cee2e411c9715c855f6753b84%7C1%7C0%7C636946716901390297&amp;sdata=1WKvXh7ObJbAioTzfDZmRZq7lD%2Fr144ammjI6I%2FYEnU%3D&amp;reserved=0) by Nadav Y. Arbel, Ayellet Tal and Lihi-Zelnik Manor. [Video](https://youtu.be/vxYan65O_WU)

The movement from sparse to dense correspondence has been done by adjusting to code of [[1]](https://github.com/orlitany/FSPM) which [resides here](./FSPM/) to accept sparse correspondence
---

## Installation Instructions
Made in windows 10
### Prerequisites
 - [PCL(1.8.1 64 bit MSVC 2017)](https://github.com/PointCloudLibrary/pcl/releases)
   - Might have to add VTK_DIR to environment variables : <PCL_intallation_folder>\3rdParty\VTK\lib\cmake\vtk­7.0
 - CMAKE
 - matlab
 - Visual Studio 2017 64 bit 

### Compilation
1. Open cmake
2. set source code directory to: <this_repository_root>\nnf_sparse_code\
3. set build dir to <source_dir>\build\
4. configure (with Visual Studio 15 2017 Win64) and generate
5. click on open project
6. in Visual Studio make sure branch is set to release and build 3DDIS_OMP

You now should be able to run everything in the root dir

---
## Evaluation
We provide instructions to replicate our results. Sparse correspondences reside in "<match_dir>\refined_sparse\", while dense correspondences reside in "<match_dir>\dense\"

### SHREC'16:PARTIAL MATCHING OF DEFORMABLE SHAPES[2]
To run this experiment please download the test set and evaluation code from
[their site](http://www.dais.unive.it/~shrec2016/).

Run [runSHREC16A](./runSHREC16A.m) - just set the correct paths to the dataset inside the file.

For ground truth correspondences please contact the competition organizers, they had kindly provided me with the data, but had I have no permission to further divulge it myself.

Alternatively our obtained correspondences can be downloaded below.

[Sparse](./Results/SHREC16_partial_results_refined_sparse.rar), [Dense](./Results/SHREC16_partial_results_dense.rar)

### SHREC'16:Matching of Deformable Shapes with Topological Noise[3]
To run this experiment please download the  low resolution test set and evaluation code from
[their site](https://vision.in.tum.de/~laehner/shrec2016/dataset.php)

Run [runSHREC16B](./runSHREC16B.m) - just set the correct paths to the dataset inside the file.

Alternatively our obtained correspondences can be downloaded below.

[Sparse](./Results/topology_sparse.rar), [Dense](./Results/topology_dense.rar)

Here you need to provide geoedesic distance matrices for each model. I provide a [matlab code](./create_geo_matrices.m) which achieves this.

### FAUST[4]

Download the dataset from [the site](http://faust.is.tue.mpg.de/).

Since we require sampling to make the problem tractable, and the dense correspondence algorithm[1] requires 2-manifold meshes of a specific area, you should copy the code from [here](./Faust_Sampling) into the model directory and run "run.bat"

Then run Run [run_FAUST_test](./run_FAUST_test.m) - just set the correct paths inside the file.

We have only made a qualitative evaluation here, as it contains full to part problems

---

## Visualization
to visualizing correspondences we provide an example script for SHREC'16:Partial[1] one code is for [sparse correspondences](./visualize_sparse_matches.m), and another for [dense correspondences](./visualize_dense_matches.m)

--- 
## References
[1][Litany O, Rodolà E, Bronstein AM, Bronstein MM. "Fully spectral partial shape matching." InComputer Graphics Forum 2017 May (Vol. 36, No. 2, pp. 247-258).](http://vision.in.tum.de/_media/spezial/bib/litany-eg17.pdf)

[2][Cosmo L, Rodolà E, Bronstein MM, Torsello A, Cremers D, Sahillioglu Y. "SHREC’16: Partial matching of deformable shapes." Proc. 3DOR. 2016;2(9):12.](http://www.dais.unive.it/~shrec2016/shrec16-partial.pdf)

[3][Lähner Z, Rodolà E, Bronstein MM, Cremers D, Burghard O, Cosmo L, Dieckmann A, Klein R, Sahillioglu Y. "SHREC’16: Matching of deformable shapes with topological noise." Proc. 3DOR. 2016 May 8;2:11.](https://vision.in.tum.de/~laehner/shrec2016/shrec16topology.pdf)

[4][Bogo F, Romero J, Loper M, Black MJ. FAUST: Dataset and evaluation for 3D mesh registration. InProceedings of the IEEE Conference on Computer Vision and Pattern Recognition 2014 (pp. 3794-3801).](http://files.is.tue.mpg.de/black/papers/FAUST2014.pdf)
