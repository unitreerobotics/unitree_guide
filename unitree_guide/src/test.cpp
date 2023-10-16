#include "control/BalanceCtrl.h"
#include "common/timeMarker.h"
#include "common/unitreeRobot.h"
#include "common/PyPlot.h"
#include "Gait/WaveGenerator.h"
#include "common/mathTools.h"
#include "control/Estimator.h"

/********************************/
/*****         Test         *****/
/********************************/
// PyPlot多个静态图测试
int main(){
    PyPlot plot;
    plot.addPlot("test", 1, std::vector<std::string>{"y"});                         // 输入标量
    plot.addPlot("testArray", 3, std::vector<std::string>{"y1", "y2", "y3"});       // 输入数组
    plot.addPlot("testEigen", 4, std::vector<std::string>{"y1", "y2", "y3", "y4"}); // 输入Eigen格式的向量
    plot.addPlot("testVector", 5);                                                  // 输入Vector格式的向量，如果不指定各个物理量的名称，则默认用数字依次编号

    float array[3];
    Eigen::Matrix<double, 4, 1> eigenVec; eigenVec.setZero();
    std::vector<int> vec(5);

    for(int i(0); i < 50; ++i){
        plot.addFrame("test", i, i+1);  // 给对应名称的Plot输入数据，三个参数分别为图名、x轴坐标，数据值。输入数据的类型可以为标量、数组、Eigen格式的向量、Vector格式的向量

        array[0] = i-10;
        array[1] = i;
        array[2] = i+10;
        plot.addFrame("testArray", i, array);

        eigenVec << i-10, i, i+10, i+20;
        plot.addFrame("testEigen", eigenVec);   // 不给定x轴坐标的话，可以默认以运行该函数的时间为x轴坐标

        vec[0] = i-20;
        vec[1] = i-10;
        vec[2] = i;
        vec[3] = i+10;
        vec[4] = i+20;
        plot.addFrame("testVector", vec);

        usleep(100);
    }

    // plot.showPlot("test");                                           // 显示某个Plot
    // plot.showPlot("testArray");                                      
    // plot.showPlot(std::vector<std::string>{"test", "testVector"});   // 显示多个Plot
    plot.showPlotAll();                                                 // 显示全部Plot
}


// /*pyPlot时间对齐*/
// int main(){
//     PyPlot plt;
//     plt.addPlot("lead", 1);
//     plt.addPlot("lag",  1);

//     for(int i(0); i<1000; ++i){
//         plt.addFrame("lead", i);
//         if(i>500){
//         plt.addFrame("lag", i);
//         }
//         usleep(1000);
//     }

//     plt.showPlotAll();
// }

// /* Eigen, Matrix - Vector */
// int main(){
//     Mat3 mat;
//     Vec3 vec;

//     mat.setZero();
//     vec.setIdentity();

//     // std::cout << "result: " << mat - vec << std::endl;
// }

// /*Class array size*/
// int main(){
//     Estimator* estTest[6];
//     std::cout << "size: " << sizeof(estTest)/sizeof(estTest[0]) << std::endl;
// }

// /* windowFunc */
// int main(){
//     double a1 = windowFunc(0.1*M_PI, M_PI, 1.0, 0.4);
//     double a2 = windowFunc(0.2*M_PI, M_PI, 1.0, 0.4);
//     double a3 = windowFunc(0.6*M_PI, M_PI, 1.0, 0.4);
//     double a4 = windowFunc(0.9*M_PI, M_PI, 1.0, 0.4);

//     std::cout << a1 << ", " << a2 << ", " << a3 << ", " << a4 << std::endl;
// }


// /*updateAvgCov*/
// int main(){
//     Vec3 p0, p1, p2, p3;
//     Vec3 exp;
//     Mat3 Cp;

//     p0 << 0, 2, 4;
//     p1 << -0.2, 1.6, 2.5;
//     p2 <<  0.01, 2.11, 5.80;
//     p3 << 0.13, 1.90, 4.30;

//     AvgCov test(3, "test", 1, 0, 1);
//     test.measure(p0);
//     test.measure(p1);
//     test.measure(p2);
//     test.measure(p3);

//     // for(int i(0); i<4; ++i){
//     //     std::cout << "p: " << p[i] << std::endl;    //注释掉会计算错误，不明原因
//     //     // updateAvgCov(Cp, exp, p[i], i+1);
//     //     test.measure(p[i]);
//     // }
//     // std::cout << "exp: " << std::endl << exp << std::endl;
//     // std::cout << "Cp: " << std::endl << Cp << std::endl;

// }



// // WaveGenerator
// int main(){
//     WaveGenerator *wave;
//     Vec4 phase; 
//     VecInt4 contact;

//     wave = new WaveGenerator(0.3, 0.5, Vec4(0, 1, 1, 0));

//     // wave->calcContactPhase(phase, contact);
//     // std::cout << "contact: " << contact.transpose() << std::endl;
//     // std::cout << "phase: " << phase.transpose() << std::endl;

//     for(int i(0); i<5; ++i){
//         wave->calcContactPhase(phase, contact);
//         usleep(2000);
// std::cout << "contact: " << contact.transpose() << std::endl;
// std::cout << "phase: " << phase.transpose() << std::endl;
//     }

//     delete wave;

//     return 0;
// }


// // Test rotMatToExp
// int main(){
//     RotMat rm = rotx(M_PI);
//     Vec3 w = rotMatToExp(rm);

//     std::cout << "rm:" << std::endl << rm << std::endl;
//     std::cout << "w: " << w.transpose() << std::endl;
// }



// // Test leg kinematics
// int main(){
//     std::cout << std::fixed << std::setprecision(3);

//     A1Robot robModel;
//     Vec34 feetPos;
//     feetPos <<  0.180,  0.180, -0.180, -0.180,
//             -0.131,  0.131, -0.131,  0.131,
//             -0.200, -0.400, -0.400, -0.400;
//     Vec12 q = robModel.getQ(feetPos, FrameType::BODY);
//     LowlevelState state;
//     for(int i(0); i<12; ++i){
//         state.motorState[i].q = q(i);
//     }
//     Vec34 calFeetPos = robModel.getFeet2BPositions(state, FrameType::BODY);

//     std::cout << "feetPos: " << std::endl << feetPos << std::endl;
//     std::cout << "state q: " << std::endl << state.getQ() << std::endl;
//     std::cout << "calFeetPos: " << std::endl << calFeetPos << std::endl;



//     // A1Leg leg(0, Vec3( 0.1805, -0.047, 0));
//     // Vec3 ql = leg.calcQ(Vec3(0.18, -0.131, -0.6), FrameType::BODY);

//     // ql.setZero();
//     // Vec3 pl = leg.calcPEe2B(ql);
//     // std::cout << "ql: " << ql.transpose() << std::endl;
//     // std::cout << "pl: " << pl.transpose() << std::endl;
// }


/********************************/
/*****         Test         *****/
/********************************/
// // test rotMatToExp
// int main(){
//     Mat3 rm;
//     rm.setIdentity();

//     Vec3 so3 = rotMatToExp(rm);

//     std::cout << "so3: " << so3.transpose() << std::endl;
//     std::cout << "1/sin: " << 1.0f/sin(1e-6) << std::endl;
//     std::cout << "acos 0: " << acos(0) << std::endl;
//     std::cout << "acos 1: " << acos(1) << std::endl;
// }


/********************************/
/*****         Test         *****/
/********************************/
// // test balance controller
// int main(){
//     // double mass = 1;
//     // Mat3 Ig;
//     // Mat6 S;
//     // double alpha = 0.01;
//     // double beta  = 0.01;
//     // Ig << 0.1, 0, 0,
//     //       0, 0.1, 0,
//     //       0, 0, 0.1;
//     // S << 1, 0, 0, 0, 0, 0,
//     //      0, 1, 0, 0, 0, 0,
//     //      0, 0, 1, 0, 0, 0,
//     //      0, 0, 0, 1, 0, 0,
//     //      0, 0, 0, 0, 1, 0,
//     //      0, 0, 0, 0, 0, 1;

//     // BalanceCtrl bCtrl(mass, Ig, S, alpha, beta);
//     BalanceCtrl bCtrl;

//     Vec34 F;
//     Vec3 ddPcd, dwbd;
//     Vec34 feetPos2B;
//     VecInt4 contact;
//     RotMat rotM;
//     double x = 0.1805;
//     double y = 0.047;
//     double h = 0.3;

//     ddPcd << 0.0, 0.0, 0.0;
//     dwbd  << 0.0, 0.0, 0.0;

//     // ddPcd.setZero();
//     // dwbd.setZero();

//     feetPos2B <<  x,  x, -x, -x,
//                  -y,  y, -y,  y,
//                  -h, -h, -h, -h;
//     contact << 1, 1, 1, 1;

//     rotM.setIdentity();
//     // rotM = rotz(M_PI/4);

//     // long long startT = getSystemTime();
//     F = bCtrl.calF(ddPcd, dwbd, rotM, rotM*feetPos2B, contact);
//     std::cout << "F: " << std::endl << F << std::endl;

//     // F = bCtrl.calF(ddPcd, dwbd, rotM, feetPos2B, contact);
//     // std::cout << "F: " << std::endl << F << std::endl;
//     // F = bCtrl.calF(ddPcd, dwbd, rotM, feetPos2B, contact);
//     // std::cout << "F: " << std::endl << F << std::endl;
//     // F = bCtrl.calF(ddPcd, dwbd, rotM, feetPos2B, contact);
//     // std::cout << "F: " << std::endl << F << std::endl;
//     // F = bCtrl.calF(ddPcd, dwbd, rotM, feetPos2B, contact);
//     // std::cout << "F: " << std::endl << F << std::endl;

//     A1Robot robModel;
//     Vec12 q;
//     q << 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3;
//     Vec12 tau = robModel.getTau(q, -F);

//     // std::cout << "tau: " << tau.transpose() << std::endl;

//     Eigen::Matrix<double, 6, 12> A;
//     A.block(0, 0, 3, 3) = I3;
//     A.block(0, 3, 3, 3) = I3;
//     A.block(0, 6, 3, 3) = I3;
//     A.block(0, 9, 3, 3) = I3;
//     A.block(3, 0, 3, 3) = skew(rotM*feetPos2B.col(0));
//     A.block(3, 3, 3, 3) = skew(rotM*feetPos2B.col(1));
//     A.block(3, 6, 3, 3) = skew(rotM*feetPos2B.col(2));
//     A.block(3, 9, 3, 3) = skew(rotM*feetPos2B.col(3));

//     std::cout << "Fb: " << (A * vec34ToVec12(F)).transpose() << std::endl;
    
//     return 0;
// }

/********************************/
/*****         multi-anime         *****/
/********************************/
// class AnimePlot{
// public:
//     AnimePlot(std::string plotName);
//     ~AnimePlot();
//     void addFrame(double x, double y);
// private:
//     Curve _curve;
//     plt::Plot plot;
//     void _threadFunc();
//     std::thread _animeThread;
//     long long _pointNum;
//     bool _start;
// };

// AnimePlot::AnimePlot(std::string plotName)
//           :plot(plotName){
//     plt::figure();
//     plt::plot();
// }

// AnimePlot::~AnimePlot(){
//     _animeThread.join();
// }

// void AnimePlot::addFrame(double x, double y){
//     if(!_start){
//         _start = true;
//         _animeThread = std::thread(&AnimePlot::_threadFunc, this);
//     }
//     _curve.x.push_back(x);
//     _curve.y.push_back(y);
// }

// void AnimePlot::_threadFunc(){
//     while(true){
//         // Just update data for this plot.
//         plot.update(_curve.x, _curve.y);

//         // Small pause so the viewer has a chance to enjoy the animation.
//         plt::pause(0.1);
//     }
// }

// // PyPlot多个动态图测试
// int main(){
//     int n = 1000;
// 	AnimePlot plot1("p1");
//     // AnimePlot plot2;

// 	for(int i=0; i<n; i++) {

// 		// x.push_back(i*i);
// 		// y.push_back(sin(2*M_PI*i/360.0));
// 		// z.push_back(log(i));
// std::cout << "i: " << i << std::endl;
// 		plot1.addFrame(i*i, sin(2*M_PI*i/360.0));
//         // plot2.addFrame(i*i, log(i));
//         usleep(2000);
// 	}
// }

/********************************/
/*****         update-anime         *****/
/********************************/
// void update_window(const double x, const double y, const double t,
//                    std::vector<double> &xt, std::vector<double> &yt)
// {
//     const double target_length = 300;
//     const double half_win = (target_length/(2.*sqrt(1.+t*t)));

//     xt[0] = x - half_win;
//     xt[1] = x + half_win;
//     yt[0] = y - half_win*t;
//     yt[1] = y + half_win*t;
// }


// int main()
// {
//     size_t n = 1000;
//     std::vector<double> x, y;

//     const double w = 0.05;
//     const double a = n/2;

//     for (size_t i=0; i<n; i++) {
//         x.push_back(i);
//         y.push_back(a*sin(w*i));
//     }

//     std::vector<double> xt(2), yt(2);

//     plt::title("Tangent of a sine curve");
//     plt::xlim(x.front(), x.back());
//     plt::ylim(-a, a);
//     plt::axis("equal");

//     // Plot sin once and for all.
//     plt::named_plot("sin", x, y);

//     // Prepare plotting the tangent.
//     plt::Plot plot("tangent");

//     plt::legend();

//     for (size_t i=0; i<n; i++) {
//         if (i % 10 == 0) {
//             update_window(x[i], y[i], a*w*cos(w*x[i]), xt, yt);

//             // Just update data for this plot.
//             plot.update(xt, yt);

//             // Small pause so the viewer has a chance to enjoy the animation.
//             plt::pause(0.1);
//             // usleep(100000);
//         }
//    }
// }