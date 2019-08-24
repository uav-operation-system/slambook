#To build the code : 
mkdir build

cd ./build

cmake ..

make

#How to run the code :

cd ./build

./ceres_customBundle -input ../data/problem-.....txt

#see more detail settings by :
./ceres_customBundle -help

BUG!!!

Please comment this code line out

/home/phillweston/git-repository/slambook/ch10/ceres_custombundle/ceresBundle.cpp:17:14: error: 
      no member named 'num_linear_solver_threads' in 'ceres::Solver::Options'
    options->num_linear_solver_threads = params.num_threads;

    ~~~~~~~  ^
1 error generated.
    ~~~~~~~