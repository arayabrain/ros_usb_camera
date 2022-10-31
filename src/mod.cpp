#include "mod.hpp"
#include <opencv2/highgui/highgui_c.h>

//Int to string
string itos(int i)
{
    stringstream s;
    s << i;
    return s.str();
}

//Judge if point is in ROI
bool MOD::ROI_mod(int x1, int y1)
{
    if (x1 >= width * w_div && x1 <= width - width * w_div && y1 >= height * h_div && y1 <= height - height * h_div)
        return true;
    return false;
}

//Image pre-processing
void MOD::pre_process()
{
    Harris_num = 0;
    F_prepoint.clear();
    F_nextpoint.clear();
    height = frame.rows*scale;
    width = frame.cols*scale;
    dsize = Size(frame.cols*scale, frame.rows*scale);
    img_scale = Mat(dsize, CV_32SC3);
    img_temp = Mat(dsize, CV_32SC3);
    img_scale = frame.clone();
    img_temp = frame.clone();
    cvtColor(img_scale, gray, cv::COLOR_BGR2GRAY);
    if(mode == UNDISTORT_IMG)
    {
        undist -> undistort_img(gray, gray);
        imshow("oh gray",gray);
    }
    //Detection square's width
    rec_width = frame.cols / 16;

    //Current frame number
    // cout << " Frame No. :   " << cal << endl;
    return;
}


void MOD::process()
{
    cap>>frame;
    if(mode!=NONE)
        undist->undistort_map(frame);

    double t = (double)cvGetTickCount();
    if (frame.empty()) return;
    cal++;
    pre_process();
    //Process every margin
    if (cal % margin != 0) return;
    detect_aruco_marker();
    if (prevgray.data)
    {
        //calcOpticalFlowPyrLK
        goodFeaturesToTrack(prevgray, prepoint, 1000, 0.01, 8, Mat(), 3, true, 0.04);
        if(mode ==UNDISTORT_PTS)
            undist ->undistort_pts(prepoint);
        if(prepoint.size() < 5){
            std::swap(prevgray, gray);
            cv::resize(img_temp, pre_frame, dsize);
            return;
        }
        cornerSubPix(prevgray, prepoint, Size(10, 10), Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
        calcOpticalFlowPyrLK(prevgray, gray, prepoint, nextpoint, state, err, Size(22, 22), 5, TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01));
        optical_flow_check();

        //Find corners
        for (int i = 0; i < state.size(); i++)
        {
            double x1 = prepoint[i].x, y1 = prepoint[i].y;
            double x2 = nextpoint[i].x, y2 = nextpoint[i].y;
            if (state[i] != 0)
            {
                //Draw all corners
                circle(img_scale, nextpoint[i], 3, Scalar(255, 0, 255));
                circle(pre_frame, prepoint[i], 2, Scalar(255, 0, 255));
            }
        }
        if (Harris_num <= 6) {
            std::swap(prevgray, gray);
            cv::resize(img_temp, pre_frame, dsize);
            return;
        }
        //F-Matrix
        vector<Point2f> F2_prepoint, F2_nextpoint;
        F2_prepoint.clear();
        F2_nextpoint.clear();
        double errs = 0;
        Mat F = findFundamentalMat(F_prepoint, F_nextpoint, mask, FM_RANSAC, thre_RANSAC, 0.99);
        for (int i = 0; i < mask.rows; i++)
        {
            if (mask.at<uchar>(i, 0) == 0);
            else
            {
                double A = F.at<double>(0, 0)*F_prepoint[i].x + F.at<double>(0, 1)*F_prepoint[i].y + F.at<double>(0, 2);
                double B = F.at<double>(1, 0)*F_prepoint[i].x + F.at<double>(1, 1)*F_prepoint[i].y + F.at<double>(1, 2);
                double C = F.at<double>(2, 0)*F_prepoint[i].x + F.at<double>(2, 1)*F_prepoint[i].y + F.at<double>(2, 2);
                double dd = fabs(A*F_nextpoint[i].x + B*F_nextpoint[i].y + C) / sqrt(A*A + B*B);
                errs += dd;
                if (dd > 0.1)
                    circle(pre_frame, F_prepoint[i], 6, Scalar(255, 0, 0), 3);
                else
                {
                    F2_prepoint.push_back(F_prepoint[i]);
                    F2_nextpoint.push_back(F_nextpoint[i]);
                }
            }
        }

        F_prepoint = F2_prepoint;
        F_nextpoint = F2_nextpoint;
        // cout << "Errors in total is " << errs << "pixels" << endl;

        T.clear();
        for (int i = 0; i < prepoint.size(); i++)
        {
            if (state[i] != 0)
            {
                double A = F.at<double>(0, 0)*prepoint[i].x + F.at<double>(0, 1)*prepoint[i].y + F.at<double>(0, 2);
                double B = F.at<double>(1, 0)*prepoint[i].x + F.at<double>(1, 1)*prepoint[i].y + F.at<double>(1, 2);
                double C = F.at<double>(2, 0)*prepoint[i].x + F.at<double>(2, 1)*prepoint[i].y + F.at<double>(2, 2);
                double dd = fabs(A*nextpoint[i].x + B*nextpoint[i].y + C) / sqrt(A*A + B*B);
                line(img_scale, Point((int)prepoint[i].x, (int)prepoint[i].y), Point((int)nextpoint[i].x, (int)nextpoint[i].y), Scalar{ 255, 255, 0 }, 2);
                line(pre_frame, Point((int)prepoint[i].x, (int)prepoint[i].y), Point((int)nextpoint[i].x, (int)nextpoint[i].y), Scalar{ 0, 255, 0 }, 1);

                //Judge outliers
                if (dd <= thre_dist2epipolar) continue;
                dis[T.size()] = dd;
                T.push_back(nextpoint[i]);

                //Draw outliers
                circle(pre_frame, prepoint[i], 3, Scalar(255, 255, 255), 2);

                //Epipolar lines
                if (fabs(B) < 0.0001)
                {
                    double xx = C / A, yy = 0;
                    double xxx = C / A, yyy = gray.cols;
                    line(pre_frame, Point(xx, yy), Point(xxx, yyy), Scalar::all(-1), 0.01);
                    continue;
                }
                double xx = 0, yy = -C / B;
                double xxx = gray.cols, yyy = -(C + A*gray.cols) / B;
                if (fabs(yy) > 12345 || fabs(yyy) > 12345)
                {
                    yy = 0;
                    xx = -C / A;
                    yyy = gray.rows;
                    xxx = -(C + B*yyy) / A;
                }
                line(img_scale, Point(xx, yy), Point(xxx, yyy), Scalar::all(-1), 0.01);
                line(pre_frame, Point(xx, yy), Point(xxx, yyy), Scalar::all(-1), 0.01);
            }
        }
        if(T.size() > 0) cout << "moving objects:" << T.size() << endl;
        frame = aruco_frame.clone();
        //Draw detections
        draw_detection();

        //Draw ROI
        rectangle(frame, Point(width * w_div / scale, height * h_div / scale), Point((width - width * w_div) / scale, height * (1 - h_div) / scale), Scalar(255, 0, 0), 1, 0);
        pub_image = frame.clone();
    }

    std::swap(prevgray, gray);
    cv::resize(img_temp, pre_frame, dsize);
    t = (double)cvGetTickCount() - t;
    cout << "cost time: " << t / ((double)cvGetTickFrequency()*1000.) << "ms" << endl;
}
