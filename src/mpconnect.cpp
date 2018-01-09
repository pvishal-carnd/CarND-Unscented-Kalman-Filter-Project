#include <iostream>
#include <math.h>
#include "ukf.h"
#include "tools.h"
#include "matplotlib-cpp/matplotlibcpp.h"

#include "mpconnect.h"

using namespace std;
namespace plt = matplotlibcpp;

int PltConnect(string in_file_name){

    //string in_file_name_ = argv[1];
    ifstream in_file_(in_file_name.c_str(), ifstream::in);

    //string out_file_name_ = argv[2];
    //ofstream out_file_(out_file_name_.c_str(), ofstream::out);


    //vector<MeasurementPackage> measurement_pack_list;
    //vector<GroundTruthPackage> gt_pack_list;

    string line;
    // Create a Kalman Filter instance
    UKF ukf;

    // used to compute the RMSE later
    Tools tools;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    // Plot variables
    vector<double> pGrTruthx, pGrTruthy;
    vector<double> pLidarx, pLidary;
    vector<double> pRadarx, pRadary;
    vector<double> pEstx, pEsty;
    vector<double> pRmsePx, pRmsePy, pRmseVx, pRmseVy;
    vector<double> pNisRadar, pNisLidar;

    while (getline(in_file_, line)){
        string sensor_type;

        MeasurementPackage meas_package;
        istringstream iss(line);

        long long timestamp;

        // reads first element from the current line
        iss >> sensor_type;
        if (sensor_type.compare("L") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;

            // Read the values
            iss >> px;
            iss >> py;

            // Save in a package
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;

            // Store the plot values
            pLidarx.push_back(px);
            pLidary.push_back(py);
        } else if (sensor_type.compare("R") == 0) {

            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;

            // Read the values
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;

            // Save them in a package
            meas_package.raw_measurements_ << ro,theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;

            // Store the plot values
            pRadarx.push_back(ro*cos(theta));
            pRadary.push_back(ro*sin(theta));
        }

        // Read and store the ground truth values
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        VectorXd gt_values(4);
        gt_values(0) = x_gt;
        gt_values(1) = y_gt;
        gt_values(2) = vx_gt;
        gt_values(3) = vy_gt;
        ground_truth.push_back(gt_values);

        pGrTruthx.push_back(x_gt);
        pGrTruthy.push_back(y_gt);

        //Call ProcessMeasurment(meas_package) for Kalman filter
        ukf.ProcessMeasurement(meas_package);

        //Push the current estimated x,y positon from the Kalman filter's state vector

        VectorXd estimate(4);

        double p_x = ukf.x_(0);
        double p_y = ukf.x_(1);
        double v   = ukf.x_(2);
        double yaw = ukf.x_(3);

        double vx = cos(yaw)*v;
        double vy = sin(yaw)*v;

        estimate(0) = p_x;
        estimate(1) = p_y;
        estimate(2) = vx;
        estimate(3) = vy;

        estimations.push_back(estimate);

        pEstx.push_back(p_x);
        pEsty.push_back(p_y);

        VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
        pRmsePx.push_back(RMSE(0));
        pRmsePy.push_back(RMSE(1));
        pRmseVx.push_back(RMSE(2));
        pRmseVy.push_back(RMSE(3));

        pNisRadar.push_back(ukf.NIS_radar_);
        pNisLidar.push_back(ukf.NIS_lidar_);
        //cout << RMSE << endl;
        //cout << count << endl;
    }

    if (in_file_.is_open()){
        in_file_.close();
    }

    // Compare with the data
    plt::figure();
    plt::named_plot("Lidar", pLidarx, pLidary, "bo");
    plt::named_plot("Radar", pRadarx, pRadary, "go");
    plt::named_plot("Estimation", pEstx, pEsty, "r-");
    plt::grid(1);
    plt::legend();
    //plt::named_plot("RMSE", pRmse, "b");
    //plt::show(0);
    plt::tight_layout();
    plt::save("imgs/pwData.png");

    //Compare with ground truth
    plt::figure();
    plt::named_plot("Ground truth", pGrTruthx, pGrTruthy, "r-");
    plt::named_plot("Estimation", pEstx, pEsty, "b-");
    plt::grid(1);
    plt::legend();
    //plt::show(0);
    plt::tight_layout();
    plt::save("imgs/pwGrTruth.png");

    // Plot RMSEs
    plt::figure();
    plt::subplot(4,1,1);
    plt::plot(pRmsePx, "b");
    plt::grid(1);
    plt::subplot(4,1,2);
    plt::plot(pRmsePy, "b");
    plt::grid(1);
    plt::subplot(4,1,3);
    plt::plot(pRmseVx, "b");
    plt::grid(1);
    plt::subplot(4,1,4);
    plt::plot(pRmseVy, "b");
    plt::grid(1);
    plt::save("imgs/pwRmse.png");

    // Plot NIS
    vector<double> threshLidar(500, 6);
    vector<double> threshRadar(500, 7.8);
    plt::figure();
    plt::subplot(2,1,1);
    plt::plot(pNisRadar, "b");
    plt::plot(threshRadar,"r--");
    plt::grid(1);
    plt::subplot(2,1,2);
    plt::plot(pNisLidar, "b");
    plt::plot(threshLidar,"r--");
    plt::grid(1);
    //getchar();
    plt::show();
    return 0;
}
