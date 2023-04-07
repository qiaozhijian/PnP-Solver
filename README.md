## PnP-Solver
In this project, we implement a PnP solver for a camera with a known intrinsic matrix. The solver includes DLT (Direct Linear Transform), RANSAC (Random Sample Consensus), and GN (Gauss-Newton). The solver is tested on the synthetic data.

To evaluate the performance of the solver, we compare the pose estimation results with those of OpenCV. The results are shown as follows.

```angular2html
PnP Benchmark with 0 outlier ratio
Method: OPENCV, rotation error 0.303949 deg, translation error 0.131297
Method: OPENCV_RANSAC, rotation error 0.144112 deg, translation error 0.0910249
Method: DLT, rotation error 0.142964 deg, translation error 0.0931654
Method: DLT_RANSAC, rotation error 0.188459 deg, translation error 0.219939
Method: GAUSS_NEWTON, rotation error 0.194367 deg, translation error 0.157812

PnP Benchmark with 0.2 outlier ratio
Method: OPENCV, rotation error 119.313 deg, translation error 2.72919e+07
Method: OPENCV_RANSAC, rotation error 0.163613 deg, translation error 0.0977965
Method: DLT, rotation error 148.23 deg, translation error 105.087
Method: DLT_RANSAC, rotation error 0.271609 deg, translation error 0.252496
Method: GAUSS_NEWTON, rotation error 0.208845 deg, translation error 0.155049
```

## Run the code
```angular2html
mkdir build
cd build
cmake ..
make
./pnp_benchmark
```