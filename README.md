# test-superq-giulia

This repo provides some code for comparing **finite-difference computation**
(implemented by [`superquadric-model`](https://github.com/robotology/superquadric-model/tree/test-finite-diff)
in the branch `test-fin-diff`)
vs **analytic computation** (implemented in this repo) of superquadrics.
This code is derived from [`test-superq`](https://gist.github.com/pattacini/4eb6c5e40667d1242bc3470253d765af). 

## Dependencies
- [VTK](https://github.com/Kitware/VTK)
- [Ipopt](https://github.com/coin-or/Ipopt)
- [Yarp](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)

## How to run the test
- launch `superquadric-model --from config-classes-fixed.ini`
- launch `test-superquadric  --file path-to-pointcloud-file --trials <num_trials> --visualize <0 or 1> --file_name path-to-saving-file`

The file `config-classes-fixed.ini` is available
[here](https://github.com/robotology/superquadric-model/blob/test-finite-diff/app/conf/config-classes-fixed.ini). 


The option `--file` requires passing the path to the file containing the point cloud as per the instructions than can be found in
the [find-superquadric](https://github.com/pattacini/find-superquadric) repository.

The option `--trials` refers to the number of times we want to execute the comparison. Everytime a different subset of points is sampled 
from the point cloud.

The option `--visualize` is used to enable/disable the visualization of the estimated superquadrics.

The option `--file_name` is the path to the file used for saving the statistics.

## Comparison
In order to make the comparison fair, here is the list of parameters I used and changes I introduced:

- `tol = 1e-6` for analytic, `tol = 1e-5` for finite difference
- `inside-penalty = 1` (parameter available only in the analytic implementation)
- `mu_strategy = adaptive`
- no constraints on the _z_ axis
- same initialization ([`superquadric-model`](https://github.com/robotology/superquadric-model/tree/test-finite-diff) 
on branch `test-fin-diff` now uses the same initialization of this repo).

Quantity analyzed:
- `average error`*
- `standard deviation of error`
- `average execution time`
- `standard deviation of execution time`

*This is computed by evaluating `F - 1`, where `F` is the `inside-outside` function of a superquadric (including all the angles for orientation,
i.e. with 11 parameters).

## Results
| Object | Average error | Std error | Average time | Std time | 
| --- | --- | --- | --- | --- |
| Box (FF tol=1e-5/ tol=1e-6) | 0.0847043 / 0.0214944 | 0.0493931 / 0.0068388| 0.105396 / 0.167901 | 0.0254845 / 0.0253024| 
| Box (A) | 0.0321511 | 0.0109134 | 0.12648 | 0.0352853 | 
| Car (FF) | 0.0384336 | 0.00811573 | 0.101143 | 0.0202455 | 
| Car (A) | 0.126919 | 0.0497111 | 0.1537 | 0.0547847 | 
| Cleaner (FF) | 0.0567448 | 0.0348972 | 0.163511 | 0.0563423 | 
| Cleaner (A) | 0.0779995 | 0.0821701 | 0.211987 | 0.0547847 | 
| Toy (FF) | 0.102369 | 0.0172055 | 0.15199 | 0.0295593 | 
| Toy (A) | 0.109989 | 0.0105534 | 0.211006 | 0.0382253 | 


<img src="https://github.com/giuliavezzani/test-superq-giulia/blob/master/misc/box-tol5.gif" width=150 height=150> <img src="https://github.com/giuliavezzani/test-superq-giulia/blob/master/misc/box-tol6.gif" width=150 height=150> <img src="https://github.com/giuliavezzani/test-superq-giulia/blob/master/misc/car.gif" width=150 height=150> <img src="https://github.com/giuliavezzani/test-superq-giulia/blob/master/misc/cleaner.gif" width=150 height=150> <img src="https://github.com/giuliavezzani/test-superq-giulia/blob/master/misc/toy.gif" width=150 height=150>

Superquadrics computed with finite difference approach are rendered with green color.

## Comments
Here are some comments to explain the results and justify the improvements obtained for finite difference implementation:

- The new initialization used is much better. Main differences:
  - centroid is now computed from bounding boxes and not as baricenter of the point cloud.
  - `initial orientation = 0.0 0.0 0.0`.
- Lower tolerance improves the results.
- The execution time is computed as the time required by Ipopt for estimating the superquadric and not the time required by the `rpc` communication (that is bugged and needs to be re-designed).
- Usually, superquadrics estimated with the finite difference approach are bigger.



