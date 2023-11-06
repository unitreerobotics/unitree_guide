/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "control/Estimator.h"
#include "common/mathTools.h"
#include "common/enumClass.h"

Estimator::Estimator(QuadrupedRobot *robotModel, LowlevelState* lowState, 
                     VecInt4 *contact, Vec4 *phase, double dt, Vec18 Qdig,
                     std::string testName)
          :_robModel(robotModel), _lowState(lowState), _contact(contact),
           _phase(phase), _dt(dt), _Qdig(Qdig), _estName(testName){

    _initSystem();
}

Estimator::Estimator(QuadrupedRobot *robotModel, LowlevelState* lowState, 
                     VecInt4 *contact, Vec4 *phase, double dt)
          :_robModel(robotModel), _lowState(lowState), _contact(contact), 
           _phase(phase), _dt(dt){

    for(int i(0); i<_Qdig.rows(); ++i){
        if(i < 3){
            _Qdig(i) = 0.0003;
        }
        else if(i < 6){
            _Qdig(i) = 0.0003;
        }
        else{
            _Qdig(i) = 0.01;
        }
    }

    _estName = "current";

    _initSystem();

}

Estimator::~Estimator(){
}

void Estimator::_initSystem(){
    _g << 0, 0, -9.81;
    _largeVariance = 100;

    _xhat.setZero();
    _u.setZero();
    _A.setZero();
    _A.block(0, 0, 3, 3) = I3;
    _A.block(0, 3, 3, 3) = I3 * _dt;
    _A.block(3, 3, 3, 3) = I3;
    _A.block(6, 6, 12, 12) = I12;
    _B.setZero();
    _B.block(3, 0, 3, 3) = I3 * _dt;
    _C.setZero();
    _C.block(0, 0, 3, 3) = -I3;
    _C.block(3, 0, 3, 3) = -I3;
    _C.block(6, 0, 3, 3) = -I3;
    _C.block(9, 0, 3, 3) = -I3;
    _C.block(12, 3, 3, 3) = -I3;
    _C.block(15, 3, 3, 3) = -I3;
    _C.block(18, 3, 3, 3) = -I3;
    _C.block(21, 3, 3, 3) = -I3;
    _C.block(0, 6, 12, 12) = I12;
    _C(24, 8) = 1;
    _C(25, 11) = 1;
    _C(26, 14) = 1;
    _C(27, 17) = 1;
    _P.setIdentity();
    _P = _largeVariance * _P;

    _RInit <<  0.008 , 0.012 ,-0.000 ,-0.009 , 0.012 , 0.000 , 0.009 ,-0.009 ,-0.000 ,-0.009 ,-0.009 , 0.000 ,-0.000 , 0.000 ,-0.000 , 0.000 ,-0.000 ,-0.001 ,-0.002 , 0.000 ,-0.000 ,-0.003 ,-0.000 ,-0.001 , 0.000 , 0.000 , 0.000 , 0.000,
               0.012 , 0.019 ,-0.001 ,-0.014 , 0.018 ,-0.000 , 0.014 ,-0.013 ,-0.000 ,-0.014 ,-0.014 , 0.001 ,-0.001 , 0.001 ,-0.001 , 0.000 , 0.000 ,-0.001 ,-0.003 , 0.000 ,-0.001 ,-0.004 ,-0.000 ,-0.001 , 0.000 , 0.000 , 0.000 , 0.000,
               -0.000, -0.001,  0.001,  0.001, -0.001,  0.000, -0.000,  0.000, -0.000,  0.001,  0.000, -0.000,  0.000, -0.000,  0.000,  0.000, -0.000, -0.000,  0.000, -0.000, -0.000, -0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,
               -0.009, -0.014,  0.001,  0.010, -0.013,  0.000, -0.010,  0.010,  0.000,  0.010,  0.010, -0.000,  0.001,  0.000,  0.000,  0.001, -0.000,  0.001,  0.002, -0.000,  0.000,  0.003,  0.000,  0.001,  0.000,  0.000,  0.000,  0.000,
               0.012 , 0.018 ,-0.001 ,-0.013 , 0.018 ,-0.000 , 0.013 ,-0.013 ,-0.000 ,-0.013 ,-0.013 , 0.001 ,-0.001 , 0.000 ,-0.001 , 0.000 , 0.001 ,-0.001 ,-0.003 , 0.000 ,-0.001 ,-0.004 ,-0.000 ,-0.001 , 0.000 , 0.000 , 0.000 , 0.000,
               0.000 ,-0.000 , 0.000 , 0.000 ,-0.000 , 0.001 , 0.000 , 0.000 ,-0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 , 0.000 ,-0.000 , 0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 ,-0.000 ,-0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000,
               0.009 , 0.014 ,-0.000 ,-0.010 , 0.013 , 0.000 , 0.010 ,-0.010 ,-0.000 ,-0.010 ,-0.010 , 0.000 ,-0.001 , 0.000 ,-0.001 , 0.000 ,-0.000 ,-0.001 ,-0.001 , 0.000 ,-0.000 ,-0.003 ,-0.000 ,-0.001 , 0.000 , 0.000 , 0.000 , 0.000,
               -0.009, -0.013,  0.000,  0.010, -0.013,  0.000, -0.010,  0.009,  0.000,  0.010,  0.010, -0.000,  0.001, -0.000,  0.000, -0.000,  0.000,  0.001,  0.002,  0.000,  0.000,  0.003,  0.000,  0.001,  0.000,  0.000,  0.000,  0.000,
               -0.000, -0.000, -0.000,  0.000, -0.000, -0.000, -0.000,  0.000,  0.001,  0.000,  0.000,  0.000,  0.000, -0.000,  0.000, -0.000,  0.000, -0.000,  0.000, -0.000,  0.000,  0.000, -0.000, -0.000,  0.000,  0.000,  0.000,  0.000,
               -0.009, -0.014,  0.001,  0.010, -0.013,  0.000, -0.010,  0.010,  0.000,  0.010,  0.010, -0.000,  0.001,  0.000,  0.000, -0.000, -0.000,  0.001,  0.002, -0.000,  0.000,  0.003,  0.000,  0.001,  0.000,  0.000,  0.000,  0.000,
               -0.009, -0.014,  0.000,  0.010, -0.013,  0.000, -0.010,  0.010,  0.000,  0.010,  0.010, -0.000,  0.001, -0.000,  0.000, -0.000,  0.000,  0.001,  0.002, -0.000,  0.000,  0.003,  0.001,  0.001,  0.000,  0.000,  0.000,  0.000,
               0.000 , 0.001 ,-0.000 ,-0.000 , 0.001 ,-0.000 , 0.000 ,-0.000 , 0.000 ,-0.000 ,-0.000 , 0.001 , 0.000 ,-0.000 ,-0.000 ,-0.000 , 0.000 , 0.000 ,-0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000,
               -0.000, -0.001,  0.000,  0.001, -0.001, -0.000, -0.001,  0.001,  0.000,  0.001,  0.001,  0.000,  1.708,  0.048,  0.784,  0.062,  0.042,  0.053,  0.077,  0.001, -0.061,  0.046, -0.019, -0.029,  0.000,  0.000,  0.000,  0.000,
               0.000 , 0.001 ,-0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 , 0.000 ,-0.000 ,-0.000 , 0.048 , 5.001 ,-1.631 ,-0.036 , 0.144 , 0.040 , 0.036 , 0.016 ,-0.051 ,-0.067 ,-0.024 ,-0.005 , 0.000 , 0.000 , 0.000 , 0.000,
               -0.000, -0.001,  0.000,  0.000, -0.001, -0.000, -0.001,  0.000,  0.000,  0.000,  0.000, -0.000,  0.784, -1.631,  1.242,  0.057, -0.037,  0.018,  0.034, -0.017, -0.015,  0.058, -0.021, -0.029,  0.000,  0.000,  0.000,  0.000,
               0.000 , 0.000 , 0.000 , 0.001 , 0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 ,-0.000 ,-0.000 ,-0.000 , 0.062 ,-0.036 , 0.057 , 6.228 ,-0.014 , 0.932 , 0.059 , 0.053 ,-0.069 , 0.148 , 0.015 ,-0.031 , 0.000 , 0.000 , 0.000 , 0.000,
               -0.000,  0.000, -0.000, -0.000,  0.001,  0.000, -0.000,  0.000,  0.000, -0.000,  0.000,  0.000,  0.042,  0.144, -0.037, -0.014,  3.011,  0.986,  0.076,  0.030, -0.052, -0.027,  0.057,  0.051,  0.000,  0.000,  0.000,  0.000,
               -0.001, -0.001, -0.000,  0.001, -0.001,  0.000, -0.001,  0.001, -0.000,  0.001,  0.001,  0.000,  0.053,  0.040,  0.018,  0.932,  0.986,  0.885,  0.090,  0.044, -0.055,  0.057,  0.051, -0.003,  0.000,  0.000,  0.000,  0.000,
               -0.002, -0.003,  0.000,  0.002, -0.003, -0.000, -0.001,  0.002,  0.000,  0.002,  0.002, -0.000,  0.077,  0.036,  0.034,  0.059,  0.076,  0.090,  6.230,  0.139,  0.763,  0.013, -0.019, -0.024,  0.000,  0.000,  0.000,  0.000,
               0.000 , 0.000 ,-0.000 ,-0.000 , 0.000 ,-0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 ,-0.000 , 0.000 , 0.001 , 0.016 ,-0.017 , 0.053 , 0.030 , 0.044 , 0.139 , 3.130 ,-1.128 ,-0.010 , 0.131 , 0.018 , 0.000 , 0.000 , 0.000 , 0.000,
               -0.000, -0.001, -0.000,  0.000, -0.001, -0.000, -0.000,  0.000,  0.000,  0.000,  0.000,  0.000, -0.061, -0.051, -0.015, -0.069, -0.052, -0.055,  0.763, -1.128,  0.866, -0.022, -0.053,  0.007,  0.000,  0.000,  0.000,  0.000,
               -0.003, -0.004, -0.000,  0.003, -0.004, -0.000, -0.003,  0.003,  0.000,  0.003,  0.003,  0.000,  0.046, -0.067,  0.058,  0.148, -0.027,  0.057,  0.013, -0.010, -0.022,  2.437, -0.102,  0.938,  0.000,  0.000,  0.000,  0.000,
               -0.000, -0.000,  0.000,  0.000, -0.000,  0.000, -0.000,  0.000, -0.000,  0.000,  0.001,  0.000, -0.019, -0.024, -0.021,  0.015,  0.057,  0.051, -0.019,  0.131, -0.053, -0.102,  4.944,  1.724,  0.000,  0.000,  0.000,  0.000,
               -0.001, -0.001,  0.000,  0.001, -0.001,  0.000, -0.001,  0.001, -0.000,  0.001,  0.001,  0.000, -0.029, -0.005, -0.029, -0.031,  0.051, -0.003, -0.024,  0.018,  0.007,  0.938,  1.724,  1.569,  0.000,  0.000,  0.000,  0.000,
               0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 1.0 , 0.000 , 0.000 , 0.000,
               0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 1.0 , 0.000 , 0.000,
               0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 1.0 , 0.000,
               0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 1.0;

    /* A1 Worked */
    _Cu <<   268.573,  -43.819, -147.211,
            -43.819 ,  92.949 ,  58.082,
            -147.211,   58.082,  302.120;

    _QInit = _Qdig.asDiagonal();
    _QInit +=  _B * _Cu * _B.transpose();

    _RCheck  = new AvgCov(28, _estName + " R");
    _uCheck  = new AvgCov(3,  _estName + " u");

    _vxFilter = new LPFilter(_dt, 3.0);
    _vyFilter = new LPFilter(_dt, 3.0);
    _vzFilter = new LPFilter(_dt, 3.0);


    /* ROS odometry publisher */
    #if defined(COMPILE_WITH_MOVE_BASE) && defined(COMPILE_WITH_PUBLISH_ODOM)
        _nh = rclcpp::Node::make_shared("odom_pub");
        _odom_publisher = _nh->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(rclcpp::KeepLast(1)));
    #endif  // COMPILE_WITH_MOVE_BASE && COMPILE_WITH_PUBLISH_ODOM
}

void Estimator::run(){
    _feetH.setZero();
    _feetPosGlobalKine = _robModel->getFeet2BPositions(*_lowState, FrameType::GLOBAL);
    _feetVelGlobalKine = _robModel->getFeet2BVelocities(*_lowState, FrameType::GLOBAL);

    _Q = _QInit;
    _R = _RInit;

    for(int i(0); i < 4; ++i){
        if((*_contact)(i) == 0){
            _Q.block(6+3*i, 6+3*i, 3, 3) = _largeVariance * I3;
            _R.block(12+3*i, 12+3*i, 3, 3) = _largeVariance * I3;
            _R(24+i, 24+i) = _largeVariance;
        }
        else{
            _trust = windowFunc((*_phase)(i), 0.2);
            _Q.block(6+3*i, 6+3*i, 3, 3) = (1 + (1-_trust)*_largeVariance) * _QInit.block(6+3*i, 6+3*i, 3, 3);
            _R.block(12+3*i, 12+3*i, 3, 3) = (1 + (1-_trust)*_largeVariance) * _RInit.block(12+3*i, 12+3*i, 3, 3);
            _R(24+i, 24+i) = (1 + (1-_trust)*_largeVariance) * _RInit(24+i, 24+i);
        }
        _feetPos2Body.segment(3*i, 3) = _feetPosGlobalKine.col(i);
        _feetVel2Body.segment(3*i, 3) = _feetVelGlobalKine.col(i);
    }

    _rotMatB2G = _lowState->getRotMat();
    _u = _rotMatB2G * _lowState->getAcc() + _g;
    _xhat = _A * _xhat + _B * _u;
    _yhat = _C * _xhat;
    _y << _feetPos2Body, _feetVel2Body, _feetH;

    _Ppriori = _A * _P * _A.transpose() + _Q;
    _S =  _R + _C * _Ppriori * _C.transpose();
    _Slu = _S.lu();
    _Sy = _Slu.solve(_y - _yhat);
    _Sc = _Slu.solve(_C);
    _SR = _Slu.solve(_R);
    _STC = (_S.transpose()).lu().solve(_C);
    _IKC = I18 - _Ppriori*_C.transpose()*_Sc;

    _xhat += _Ppriori * _C.transpose() * _Sy;
    _P =  _IKC * _Ppriori * _IKC.transpose()
        + _Ppriori * _C.transpose() * _SR * _STC * _Ppriori.transpose();

    _vxFilter->addValue(_xhat(3));
    _vyFilter->addValue(_xhat(4));
    _vzFilter->addValue(_xhat(5));

    #if defined(COMPILE_WITH_MOVE_BASE) && defined(COMPILE_WITH_PUBLISH_ODOM)
        if(_count % ((int)( 1.0/(_dt*_pub_freq))) == 0){
            _current_time = _nh->get_clock()->now();
            /* tf */
            _odom_tf.header.stamp = _nh->get_clock()->now();
            _odom_tf.header.frame_id = "odom";
            _odom_tf.child_frame_id  = "base";

            _odom_tf.transform.translation.x = _xhat(0);
            _odom_tf.transform.translation.y = _xhat(1);
            _odom_tf.transform.translation.z = _xhat(2);
            _odom_tf.transform.rotation.w = _lowState->imu.quaternion[0];
            _odom_tf.transform.rotation.x = _lowState->imu.quaternion[1];
            _odom_tf.transform.rotation.y = _lowState->imu.quaternion[2];
            _odom_tf.transform.rotation.z = _lowState->imu.quaternion[3];

            _odom_broadcaster->sendTransform(_odom_tf);

            /* odometry */
            _odom_msg.header.stamp = _current_time;
            _odom_msg.header.frame_id = "odom";

            _odom_msg.pose.pose.position.x = _xhat(0);
            _odom_msg.pose.pose.position.y = _xhat(1);
            _odom_msg.pose.pose.position.z = _xhat(2);

            _odom_msg.pose.pose.orientation.w = _lowState->imu.quaternion[0];
            _odom_msg.pose.pose.orientation.x = _lowState->imu.quaternion[1];
            _odom_msg.pose.pose.orientation.y = _lowState->imu.quaternion[2];
            _odom_msg.pose.pose.orientation.z = _lowState->imu.quaternion[3];
            _odom_msg.pose.covariance = _odom_pose_covariance;

            _odom_msg.child_frame_id = "base";
            _velBody = _rotMatB2G.transpose() * _xhat.segment(3, 3);
            _wBody   = _lowState->imu.getGyro();
            _odom_msg.twist.twist.linear.x = _velBody(0);
            _odom_msg.twist.twist.linear.y = _velBody(1);
            _odom_msg.twist.twist.linear.z = _velBody(2);
            _odom_msg.twist.twist.angular.x = _wBody(0);
            _odom_msg.twist.twist.angular.y = _wBody(1);
            _odom_msg.twist.twist.angular.z = _wBody(2);
            _odom_msg.twist.covariance = _odom_twist_covariance;

            _odom_publisher->publish(_odom_msg);
            _count = 1;
        }
        ++_count;
    #endif  // COMPILE_WITH_MOVE_BASE
}

Vec3 Estimator::getPosition(){
    return _xhat.segment(0, 3);
}

Vec3 Estimator::getVelocity(){
    return _xhat.segment(3, 3);
}

Vec3 Estimator::getFootPos(int i){
    return getPosition() + _lowState->getRotMat() * _robModel->getFootPosition(*_lowState, i, FrameType::BODY);
}

Vec34 Estimator::getFeetPos(){
    Vec34 feetPos;
    for(int i(0); i < 4; ++i){
        feetPos.col(i) = getFootPos(i);
    }
    return feetPos;
}

Vec34 Estimator::getFeetVel(){
    Vec34 feetVel = _robModel->getFeet2BVelocities(*_lowState, FrameType::GLOBAL);
    for(int i(0); i < 4; ++i){
        feetVel.col(i) += getVelocity();
    }
    return feetVel;
}

Vec34 Estimator::getPosFeet2BGlobal(){
    Vec34 feet2BPos;
    for(int i(0); i < 4; ++i){
        feet2BPos.col(i) = getFootPos(i) - getPosition();
    }
    return feet2BPos;
}

