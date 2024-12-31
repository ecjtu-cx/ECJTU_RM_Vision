#include "anglesolver.h"

void AngleSolver::Init(
    const std::string &paramPath, float camBiasX, float camBiasY, float camBiasZ, float gravity)
{
    // 大装甲板实际参数
    BigArmorPoint3D = {
        cv::Point3f(-BIG_ARMOR_LEN / 2.0, BIG_ARMOR_WID / 2.0, 0.0),
        cv::Point3f(-BIG_ARMOR_LEN / 2.0, -BIG_ARMOR_WID / 2.0, 0.0),
        cv::Point3f(BIG_ARMOR_LEN / 2.0, -BIG_ARMOR_WID / 2.0, 0.0),
        cv::Point3f(BIG_ARMOR_LEN / 2.0, BIG_ARMOR_WID / 2.0, 0.0)};

    // 小装甲板实际参数
    SmallArmorPoint3D = {
        cv::Point3f(-SMALL_ARMOR_LEN / 2.0, SMALL_ARMOR_WID / 2.0, 0.0),
        cv::Point3f(-SMALL_ARMOR_LEN / 2.0, -SMALL_ARMOR_WID / 2.0, 0.0),
        cv::Point3f(SMALL_ARMOR_LEN / 2.0, -SMALL_ARMOR_WID / 2.0, 0.0),
        cv::Point3f(SMALL_ARMOR_LEN / 2.0, SMALL_ARMOR_WID / 2.0, 0.0)};

    cv::FileStorage fsread;
    fsread.open(paramPath, cv::FileStorage::READ);
    if (!fsread.isOpened())
    {
        cout << "Failed to open xml";
        return;
    }

    fsread["camera_matrix"] >> camera_matrix;
    fsread["distortion_coefficients"] >> distortion_coefficients;
    fsread.release();
    // 偏移用于pnp结算出XYZ
    _cam_bias_x = camBiasX;
    _cam_bias_z = camBiasZ;
    _cam_bias_y = camBiasY;
    _gravity = gravity;
}

void AngleSolver::solve_angle(rm_auto_aim::Armor &TargetArmor)
{
    cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1); // 旋转向量
    cv::Mat rMat;                                  // 旋转矩阵
    cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1); // 平移矩阵
    vector<cv::Point2f> D2 = vector<cv::Point2f>{
        TargetArmor.vertex[0], TargetArmor.vertex[1], TargetArmor.vertex[2], TargetArmor.vertex[3]};
    switch (TargetArmor.armor_type)
    {
    case rm_auto_aim::LARGE:
        solvePnP(
            BigArmorPoint3D, D2, this->camera_matrix, this->distortion_coefficients, rVec, tVec, false,
            cv::SOLVEPNP_IPPE);
        break;
    case rm_auto_aim::SMALL:
        solvePnP(
            SmallArmorPoint3D, D2, this->camera_matrix, this->distortion_coefficients, rVec, tVec);
        break;
    default:
        break;
    }

    double x_pos = tVec.at<double>(0, 0) + _cam_bias_x;
    double y_pos = tVec.at<double>(1, 0) + _cam_bias_y;
    double z_pos = tVec.at<double>(2, 0) + _cam_bias_z;
    xyz[0] = x_pos;
    xyz[1] = y_pos;
    xyz[2] = z_pos;

    Eigen::Matrix3d rMat_eigen;

    Rodrigues(rVec, rMat);
    cv2eigen(rMat, rMat_eigen);
    // 选用拟合曲线
    this->dis = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);

    // 采用小孔模型
    if (dis > 5000) // 距离过远要写抬枪补偿（类似斜抛运动）尤其是pitch要补偿 而yaw需要通过预测敌方位置速度进行偏移
    {
        double fx = this->camera_matrix.at<double>(0, 0);
        double fy = this->camera_matrix.at<double>(1, 1);
        double cx = this->camera_matrix.at<double>(0, 2);
        double cy = this->camera_matrix.at<double>(1, 2);
        cv::Point2f pnt;
        vector<cv::Point2f> in;
        vector<cv::Point2f> out;

        in.push_back(TargetArmor.center);
        // 对像素点去畸变
        undistortPoints(
            in, out, this->camera_matrix, this->distortion_coefficients, cv::noArray(),
            this->camera_matrix);
        pnt = out.front();

        // 去畸变后的比值
        double rxNew = (pnt.x - cx) / fx;
        double ryNew = (pnt.y - cy) / fy;

        yaw = atan(rxNew) / CV_PI * 180;
        pitch = -atan(ryNew) / CV_PI * 180;

// 弹道高度补偿 pitch角度       根据不同距离的范围设置不同的抬头补偿📏：比如2-3米设置一个补偿值，3-4米设置一个补偿值…
#ifdef DEBUG_COMPENSATION

        double vx, vy, fly_time, tmp_height = y_pos, delta_height = 0, tmp_pitch, real_height,
                                 bullet_speed = 10;
        for (size_t i = 0; i < 10; i++)
        {
            tmp_pitch = atan((tmp_height) / sqrt(x_pos * x_pos + z_pos * z_pos));
            vx = bullet_speed * cos(tmp_pitch);
            vy = bullet_speed * sin(tmp_pitch);

            fly_time = dis / (vx);
            real_height = vy * fly_time - 0.5 * _gravity * pow(fly_time, 2); // h=vt-(1/2)gt2
            delta_height = y_pos - real_height;
            tmp_height += delta_height;
            cout << "iter: " << i + 1 << " " << delta_height << endl;
        }
#endif
    }
    else
    {
        double tan_pitch = y_pos / sqrt(x_pos * x_pos + z_pos * z_pos);
        double tan_yaw = x_pos / z_pos;
        pitch = atan(tan_pitch) * 180 / CV_PI;
        yaw = -atan(tan_yaw) * 180 / CV_PI;
    }
}
// 从这个接口获取目标角度 以及 水平向量xyz
void AngleSolver::GetAngle(float &pitch, float &yaw, float &distance, float XYZ[3])
{
    pitch = this->pitch + bias_pitch;
    yaw = this->yaw + bias_yaw;
    distance = this->dis + bias_dis;
    XYZ[0] = xyz[0];
    XYZ[1] = xyz[1];
    XYZ[2] = xyz[2];
}
