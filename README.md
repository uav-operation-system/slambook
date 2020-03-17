# slambook
This is the code written for my new book about visual SLAM called "14 lectures on visual SLAM" which was released in April 2017. It is highy recommended to download the code and run it in you own machine so that you can learn more efficiently and also modify it. The code is stored by chapters like "ch2" and "ch4". Note that chapter 9 is a project so I stored it in the "project" directory.

If you have any questions about the code, please add an issue so I can see it. Contact me for more information: gao dot xiang dot thu at gmail dot com.

These codes are under MIT license. You don't need permission to use it or change it. 
Please cite this book if you are doing academic work:
Xiang Gao, Tao Zhang, Yi Liu, Qinrui Yan, 14 Lectures on Visual SLAM: From Theory to Practice, Publishing House of Electronics Industry, 2017.

In LaTeX:
`` @Book{Gao2017SLAM, 
title={14 Lectures on Visual SLAM: From Theory to Practice}, 
publisher = {Publishing House of Electronics Industry},
year = {2017},
author = {Xiang Gao and Tao Zhang and Yi Liu and Qinrui Yan},
} ``

For English readers, we are currently translating this book into an online version, see [this page](https://gaoxiang12.github.io/slambook-en/) for details.

# Contents
- ch1 Preface
- ch2 Overview of SLAM & linux, cmake
- ch3 Rigid body motion & Eigen
- ch4 Lie group and Lie Algebra & Sophus
- ch5 Cameras and Images & OpenCV
- ch6 Non-linear optimization & Ceres, g2o
- ch7 Feature based Visual Odometry
- ch8 Direct (Intensity based) Visual Odometry
- ch9 Project
- ch10 Back end optimization & Ceres, g2o
- ch11 Pose graph and Factor graph & g2o, gtsam
- ch12 Loop closure & DBoW3
- ch13 Dense reconstruction & REMODE, Octomap

# slambook (中文说明)
我最近写了一本有关视觉SLAM的书籍，这是它对应的代码。书籍将会在明年春天由电子工业出版社出版。

我强烈建议你下载这个代码。书中虽然给出了一部分，但你最好在自己的机器上编译运行它们，然后对它们进行修改以获得更好的理解。这本书的代码是按章节划分的，比如第二章内容在”ch2“文件夹下。注意第九章是工程，所以我们没有”ch9“这个文件夹，而是在”project“中存储它。

如果你在运行代码中发现问题，请在这里提交一个issue，我就能看到它。如果你有更多的问题，请给我发邮件：gaoxiang12 dot mails dot tsinghua dot edu dot cn.

本书代码使用MIT许可。使用或修改、发布都不必经过我的同意。不过，如果你是在学术工作中使用它，建议你引用本书作为参考文献。

引用格式：
高翔, 张涛, 颜沁睿, 刘毅, 视觉SLAM十四讲：从理论到实践, 电子工业出版社, 2017

LaTeX格式:
`` @Book{Gao2017SLAM, 
title={视觉SLAM十四讲：从理论到实践}, 
publisher = {电子工业出版社},
year = {2017},
author = {高翔 and 张涛 and 刘毅 and 颜沁睿},
lang = {zh}
} ``

# 目录
- ch2 概述，cmake基础
- ch3 Eigen，三维几何
- ch4 Sophus，李群与李代数
- ch5 OpenCV，图像与相机模型
- ch6 Ceres and g2o，非线性优化
- ch7 特征点法视觉里程计
- ch8 直接法视觉里程计
- ch9 project
- ch10 Ceres and g2o，后端优化1
- ch11 g2o and gtsam，位姿图优化
- ch12 DBoW3，词袋方法
- ch13 稠密地图构建

关于勘误，请参照本代码根目录下的errata.xlsx文件。此文件包含本书从第一次印刷至现在的勘误信息。勘误将随着书籍的印刷版本更新。

# g2o版本更改导致源码需要修改

1. 第７讲

1.1 pose_estimation_3d3d.cpp片段

修改版本：

```java
// 初始化g2o
typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose维度为 6, landmark 维度为 3
//Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器
std::unique_ptr<Block::LinearSolverType> linearSolver (new g2o::LinearSolverEigen<Block::PoseMatrixType>());
//Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
std::unique_ptr<Block> solver_ptr (new Block (std::move(linearSolver)));
g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::move(solver_ptr) );
//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::move(solver_ptr) );
g2o::SparseOptimizer optimizer;
optimizer.setAlgorithm( solver );
```

1.2 pose_estimation_3d2d.cpp片段

修改版本：

```java
// 初始化g2o
typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverCSparse<Block::PoseMatrixType>()); // 线性方程求解器
std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));
g2o::SparseOptimizer optimizer;
optimizer.setAlgorithm ( solver );
```

原始版本：

```java
// typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
// Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
// Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
// g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
// g2o::SparseOptimizer optimizer;
// optimizer.setAlgorithm ( solver );
```

2. 第10讲

2.1 g2o_bundle.cpp

```java
#include <Eigen/StdVector>
#include <Eigen/Core>

#include <iostream>
#include <stdint.h>

#include <unordered_set>
#include <memory>
#include <vector>
#include <stdlib.h> 

#include "g2o/stuff/sampler.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include "g2o/solvers/structure_only/structure_only_solver.h"

#include "common/BundleParams.h"
#include "common/BALProblem.h"
#include "g2o_bal_class.h"


using namespace Eigen;
using namespace std;

typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;
typedef g2o::BlockSolver<g2o::BlockSolverTraits<9,3> > BalBlockSolver;
typedef g2o::LinearSolverDense<BalBlockSolver::PoseMatrixType> BalDenseSolver;

// set up the vertexs and edges for the bundle adjustment. 
void BuildProblem(const BALProblem* bal_problem, g2o::SparseOptimizer* optimizer, const BundleParams& params)
{
    const int num_points = bal_problem->num_points();
    const int num_cameras = bal_problem->num_cameras();
    const int camera_block_size = bal_problem->camera_block_size();
    const int point_block_size = bal_problem->point_block_size();

    // Set camera vertex with initial value in the dataset.
    const double* raw_cameras = bal_problem->cameras();
    for(int i = 0; i < num_cameras; ++i)
    {
        ConstVectorRef temVecCamera(raw_cameras + camera_block_size * i,camera_block_size);
        VertexCameraBAL* pCamera = new VertexCameraBAL();
        pCamera->setEstimate(temVecCamera);   // initial value for the camera i..
        pCamera->setId(i);                    // set id for each camera vertex 
  
        // remeber to add vertex into optimizer..
        optimizer->addVertex(pCamera);
        
    }

    // Set point vertex with initial value in the dataset. 
    const double* raw_points = bal_problem->points();
    // const int point_block_size = bal_problem->point_block_size();
    for(int j = 0; j < num_points; ++j)
    {
        ConstVectorRef temVecPoint(raw_points + point_block_size * j, point_block_size);
        VertexPointBAL* pPoint = new VertexPointBAL();
        pPoint->setEstimate(temVecPoint);   // initial value for the point i..
        pPoint->setId(j + num_cameras);     // each vertex should have an unique id, no matter it is a camera vertex, or a point vertex 

        // remeber to add vertex into optimizer..
        pPoint->setMarginalized(true);
        optimizer->addVertex(pPoint);
    }

    // Set edges for graph..
    const int  num_observations = bal_problem->num_observations();
    const double* observations = bal_problem->observations();   // pointer for the first observation..

    for(int i = 0; i < num_observations; ++i)
    {
        EdgeObservationBAL* bal_edge = new EdgeObservationBAL();

        const int camera_id = bal_problem->camera_index()[i]; // get id for the camera; 
        const int point_id = bal_problem->point_index()[i] + num_cameras; // get id for the point 
        
        if(params.robustify)
        {
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            rk->setDelta(1.0);
            bal_edge->setRobustKernel(rk);
        }
        // set the vertex by the ids for an edge observation
        bal_edge->setVertex(0,dynamic_cast<VertexCameraBAL*>(optimizer->vertex(camera_id)));
        bal_edge->setVertex(1,dynamic_cast<VertexPointBAL*>(optimizer->vertex(point_id)));
        bal_edge->setInformation(Eigen::Matrix2d::Identity());
        bal_edge->setMeasurement(Eigen::Vector2d(observations[2*i+0],observations[2*i + 1]));

       optimizer->addEdge(bal_edge) ;
    }

}

void WriteToBALProblem(BALProblem* bal_problem, g2o::SparseOptimizer* optimizer)
{
    const int num_points = bal_problem->num_points();
    const int num_cameras = bal_problem->num_cameras();
    const int camera_block_size = bal_problem->camera_block_size();
    const int point_block_size = bal_problem->point_block_size();

    double* raw_cameras = bal_problem->mutable_cameras();
    for(int i = 0; i < num_cameras; ++i)
    {
        VertexCameraBAL* pCamera = dynamic_cast<VertexCameraBAL*>(optimizer->vertex(i));
        Eigen::VectorXd NewCameraVec = pCamera->estimate();
        memcpy(raw_cameras + i * camera_block_size, NewCameraVec.data(), sizeof(double) * camera_block_size);
    }

    double* raw_points = bal_problem->mutable_points();
    for(int j = 0; j < num_points; ++j)
    {
        VertexPointBAL* pPoint = dynamic_cast<VertexPointBAL*>(optimizer->vertex(j + num_cameras));
        Eigen::Vector3d NewPointVec = pPoint->estimate();
        memcpy(raw_points + j * point_block_size, NewPointVec.data(), sizeof(double) * point_block_size);
    }
}

//this function is  unused yet..
void SetMinimizerOptions(std::unique_ptr<BalBlockSolver> solver_ptr, const BundleParams& params, g2o::SparseOptimizer* optimizer)
{
    //std::cout<<"Set Minimizer  .."<< std::endl;
    g2o::OptimizationAlgorithmWithHessian* solver;
    if(params.trust_region_strategy == "levenberg_marquardt"){
        solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    }
    else if(params.trust_region_strategy == "dogleg"){
        solver = new g2o::OptimizationAlgorithmDogleg(std::move(solver_ptr));
    }
    else 
    {
        std::cout << "Please check your trust_region_strategy parameter again.."<< std::endl;
        exit(EXIT_FAILURE);
    }

    optimizer->setAlgorithm(solver);
    //std::cout<<"Set Minimizer  .."<< std::endl;
}

//this function is  unused yet..
void SetLinearSolver(std::unique_ptr<BalBlockSolver> &solver_ptr, const BundleParams& params)
{
    
    std::unique_ptr<g2o::LinearSolver<BalBlockSolver::PoseMatrixType>> linearSolver;

    if(params.linear_solver == "dense_schur" ){
        linearSolver = g2o::make_unique<BalDenseSolver>();
    }
    else if(params.linear_solver == "sparse_schur"){
        auto cholesky = g2o::make_unique<g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>>();
        cholesky->setBlockOrdering(true);
        linearSolver = std::move(cholesky);
        
         // AMD ordering , only needed for sparse cholesky solver
    }

    solver_ptr = g2o::make_unique<BalBlockSolver>(std::move(linearSolver));
    std::cout <<  "Set Complete.."<< std::endl;
}

void SetSolverOptionsFromFlags(BALProblem* bal_problem, const BundleParams& params, g2o::SparseOptimizer* optimizer)
{   
    std::unique_ptr<BalBlockSolver> solver_ptr;    
    std::unique_ptr<BalBlockSolver::LinearSolverType> linearSolver;
    if(params.linear_solver == "dense_schur" ){
         linearSolver = g2o::make_unique<BalDenseSolver>();
    }
    else if(params.linear_solver == "sparse_schur"){
        auto cholesky = g2o::make_unique<g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>>();
        cholesky->setBlockOrdering(true);
        linearSolver = std::move(cholesky);  

        // AMD ordering , only needed for sparse cholesky solver
    }
    
    solver_ptr = g2o::make_unique<BalBlockSolver>(std::move(linearSolver));
    //SetLinearSolver(solver_ptr, params);
    //SetMinimizerOptions(solver_ptr, params, optimizer);
    g2o::OptimizationAlgorithmWithHessian* solver;
    if(params.trust_region_strategy == "levenberg_marquardt"){
        solver= new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    }
    else if(params.trust_region_strategy == "dogleg"){
        solver = new g2o::OptimizationAlgorithmDogleg(std::move(solver_ptr));
    }
    else 
    {
        std::cout << "Please check your trust_region_strategy parameter again.."<< std::endl;
        exit(EXIT_FAILURE);
    }

    optimizer->setAlgorithm(solver);
}


void SolveProblem(const char* filename, const BundleParams& params)
{
    BALProblem bal_problem(filename);

    // show some information here ...
    std::cout << "bal problem file loaded..." << std::endl;
    std::cout << "bal problem have " << bal_problem.num_cameras() << " cameras and "
              << bal_problem.num_points() << " points. " << std::endl;
    std::cout << "Forming " << bal_problem.num_observations() << " observatoins. " << std::endl;

    // store the initial 3D cloud points and camera pose..
    if(!params.initial_ply.empty()){
        bal_problem.WriteToPLYFile(params.initial_ply);
    }

    std::cout << "beginning problem..." << std::endl;    
    // add some noise for the intial value
    srand(params.random_seed);
    bal_problem.Normalize();
    bal_problem.Perturb(params.rotation_sigma, params.translation_sigma,
                        params.point_sigma);
    std::cout << "Normalization complete..." << std::endl;

    g2o::SparseOptimizer optimizer;
    SetSolverOptionsFromFlags(&bal_problem, params, &optimizer);
    BuildProblem(&bal_problem, &optimizer, params);
        
    std::cout << "begin optimizaiton .."<< std::endl;
    // perform the optimizaiton 
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(params.num_iterations);
    std::cout << "optimization complete.. "<< std::endl;
    // write the optimized data into BALProblem class
    WriteToBALProblem(&bal_problem, &optimizer);
    // write the result into a .ply file.
    if(!params.final_ply.empty()){
        bal_problem.WriteToPLYFile(params.final_ply);
    }
   
}

int main(int argc, char** argv)
{    
    BundleParams params(argc,argv);  // set the parameters here.
    if(params.input.empty()){
        std::cout << "Usage: bundle_adjuster -input <path for dataset>";
        return 1;
    }
    SolveProblem(params.input.c_str(), params);  
    return 0;
}
```

# 备注

百度云备份：[https://pan.baidu.com/s/1slDE7cL]

Videos: [https://space.bilibili.com/38737757]
