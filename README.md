## PnP-Solver
In this project, we implement a PnP solver for a camera with a known intrinsic matrix. The solver includes DLT (Direct Linear Transform), RANSAC (Random Sample Consensus), and GN (Gauss-Newton). The solver is tested on the synthetic data.

To evaluate the performance of the solver, we compare the pose estimation results with those of OpenCV. The results are shown as follows.

```angular2html
I0111 00:04:37.092092 159503 pnp_benchmark.cpp:31] PnP Benchmark with 0 outlier ratio
I0111 00:04:37.269485 159503 pnp_benchmark.cpp:129] Method: OPENCV, rotation error 0.307571 deg, translation error 0.134562
I0111 00:04:37.301229 159503 pnp_benchmark.cpp:129] Method: OPENCV_RANSAC, rotation error 0.136086 deg, translation error 0.0860776
I0111 00:04:37.314630 159503 pnp_benchmark.cpp:129] Method: DLT, rotation error 0.155364 deg, translation error 0.0853514
I0111 00:04:37.617875 159503 pnp_benchmark.cpp:129] Method: DLT_RANSAC, rotation error 0.205616 deg, translation error 0.218715
I0111 00:04:37.872629 159503 pnp_benchmark.cpp:129] Method: GAUSS_NEWTON, rotation error 0.192016 deg, translation error 0.159417
I0111 00:04:38.139861 159503 pnp_benchmark.cpp:129] Method: LM, rotation error 0.216615 deg, translation error 0.144676

PnP Benchmark with 0.2 outlier ratio
I0111 00:04:38.367627 159503 pnp_benchmark.cpp:129] Method: OPENCV, rotation error 103.723 deg, translation error 127.345
I0111 00:04:38.465721 159503 pnp_benchmark.cpp:129] Method: OPENCV_RANSAC, rotation error 0.165916 deg, translation error 0.106485
I0111 00:04:38.478801 159503 pnp_benchmark.cpp:129] Method: DLT, rotation error 142.401 deg, translation error 94.9887
I0111 00:04:40.462188 159503 pnp_benchmark.cpp:129] Method: DLT_RANSAC, rotation error 0.215103 deg, translation error 0.238203
I0111 00:04:42.377627 159503 pnp_benchmark.cpp:129] Method: GAUSS_NEWTON, rotation error 0.222669 deg, translation error 0.177306
I0111 00:04:44.432622 159503 pnp_benchmark.cpp:129] Method: LM, rotation error 0.217583 deg, translation error 0.161283
```

## Run the code
```angular2html
mkdir build
cd build
cmake ..
make
./pnp_benchmark
```