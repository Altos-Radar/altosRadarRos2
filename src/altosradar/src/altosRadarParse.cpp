/*
 * @Copyright (C) 2024 by Altos Radar. All rights reserved.
 * @Author: altos
 * @Description: Altos Radar Driver
 */

#include "altosradar/PointCloud.h"
#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <errno.h>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <cmath>
#include <netinet/in.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

using namespace std;
#define widthSet 8000
#define PORT 4040
#define vrMax 60
#define vrMin -60
#define errThr 3
#define PI 3.1415926

/// @brief Calculate the radar cross section(rcs) of single point
/// @return rcs
float rcsCal(float range, float azi, float snr, float* rcsBuf) {
    int ind = (azi * 180. / PI + 60.1) * 10.;
    float rcs = powf32(range, 2.6) * snr / 5.0e6 / rcsBuf[ind];

    return rcs;
}

/// @brief Calculate the radar ego velocity based on histogram. The assumption
/// is that the statc points are more than the moving points.
/// @return velocity
float hist(vector<POINTCLOUD> pointCloudVec, float* histBuf, float step) {
    int ind = 0;
    float vr = 0;

    for (size_t i = 0; i < pointCloudVec.size(); i++) {
        for (size_t j = 0; j < POINTNUM; j++) {
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                vr = pointCloudVec[i].point[j].doppler /
                     cos(pointCloudVec[i].point[j].azi);
                ind = (vr - vrMin) / step;

                // filter the abnormal doppler value
                if (vr > vrMax || vr < vrMin || isnan(vr)) {
                    continue;
                }

                // if (vr <= 0) {
                histBuf[ind]++;
                // }
            }
        }
    }

    return float(
               (max_element(histBuf, histBuf + (int((vrMax - vrMin) / step))) -
                histBuf)) *
               step +
           vrMin;
}

/// @brief Calculate the point cloud in euclidean coordinate based on the radar
/// data. position x,y,z, doppler h, rcs s, direction v: -1 opposite direction
/// moving, 0 static, 1 same direction moving
void calPoint(vector<POINTCLOUD> pointCloudVec,
              pcl::PointCloud<pcl::PointXYZHSV>& cloud,
              int installFlag,
              float* rcsBuf,
              float step,
              float* histBuf) {
    for (size_t i = 0; i < pointCloudVec.size(); i++) {
        for (size_t j = 0; j < POINTNUM; j++) {
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                pointCloudVec[i].point[j].ele =
                    installFlag * (pointCloudVec[i].point[j].ele);

                cloud.points[i * POINTNUM + j].x =
                    (pointCloudVec[i].point[j].range) *
                    cos(pointCloudVec[i].point[j].azi) *
                    cos(pointCloudVec[i].point[j].ele);
                cloud.points[i * POINTNUM + j].y =
                    (pointCloudVec[i].point[j].range) *
                    sin(pointCloudVec[i].point[j].azi) *
                    cos(pointCloudVec[i].point[j].ele);
                cloud.points[i * POINTNUM + j].z =
                    (pointCloudVec[i].point[j].range) *
                    sin(pointCloudVec[i].point[j].ele);
                cloud.points[i * POINTNUM + j].h =
                    pointCloudVec[i].point[j].doppler;
                cloud.points[i * POINTNUM + j].s =
                    rcsCal(pointCloudVec[i].point[j].range,
                           pointCloudVec[i].point[j].azi,
                           pointCloudVec[i].point[j].snr, rcsBuf);
            }
        }
    }
    memset(histBuf, 0, sizeof(float) * int((vrMax - vrMin) / step));
    float vrEst = hist(pointCloudVec, histBuf, step);
    float tmp;
    for (size_t i = 0; i < pointCloudVec.size(); i++) {
        for (size_t j = 0; j < POINTNUM; j++) {
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                tmp = cloud.points[i * POINTNUM + j].h -
                      vrEst * cos(pointCloudVec[i].point[j].azi);
                if (tmp < -errThr) {
                    cloud.points[i * POINTNUM + j].v = -1;
                } else if (tmp > errThr) {
                    cloud.points[i * POINTNUM + j].v = 1;
                } else {
                    cloud.points[i * POINTNUM + j].v = 0;
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    float* rcsBuf = (float*)malloc(1201 * sizeof(float));
    FILE* fp_rcs = fopen("data//rcs.dat", "rb");
    fread(rcsBuf, 1201, sizeof(float), fp_rcs);
    fclose(fp_rcs);

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("altosRadar");
    auto pub =
        node->create_publisher<sensor_msgs::msg::PointCloud2>("altosRadar", 1);
    auto markerPub = node->create_publisher<visualization_msgs::msg::Marker>(
        "points_number", 10);
    auto originPub =
        node->create_publisher<visualization_msgs::msg::Marker>("origin", 10);

    visualization_msgs::msg::Marker origin;
    origin.header.frame_id = "altosRadar";
    origin.header.stamp = node->get_clock()->now();
    origin.type = visualization_msgs::msg::Marker::SPHERE;
    origin.action = visualization_msgs::msg::Marker::ADD;

    origin.pose.position.x = 0;
    origin.pose.position.y = 0;
    origin.pose.position.z = 0;
    origin.pose.orientation.x = 0;
    origin.pose.orientation.y = 0;
    origin.pose.orientation.z = 0;
    origin.pose.orientation.w = 1;

    origin.scale.x = 3;
    origin.scale.y = 3;
    origin.scale.z = 3;
    origin.color.r = 1.0;
    origin.color.g = 1.0;
    origin.color.b = 0.0;
    origin.color.a = 1;

    visualization_msgs::msg::Marker marker;
    marker.ns = "basic_shapes";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 10;
    marker.color.b = 1.0f;
    marker.color.g = 1.0f;
    marker.color.r = 1.0f;
    marker.color.a = 1;
    geometry_msgs::msg::Pose pose;
    pose.position.x = -5.;
    pose.position.y = 0.;
    pose.position.z = 0.;

    sensor_msgs::msg::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZHSV> cloud;
    cloud.width = widthSet * 2;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);

    struct sockaddr_in addr;
    struct sockaddr_in from;

    struct ip_mreq req;
    socklen_t len = sizeof(from);

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == sockfd) {
        perror("socket");
        return 0;
    }

    struct timeval timeout = {1, 300};
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout,
               sizeof(struct timeval));
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout,
               sizeof(struct timeval));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
    if (-1 == ret) {
        perror("bind");
        return 0;
    }

    req.imr_multiaddr.s_addr = inet_addr("224.1.2.4");
    req.imr_interface.s_addr = inet_addr("192.168.3.1");

    ret = setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req));
    if (ret < 0) {
        perror("setsockopt");
        return 0;
    }

    int installFlag = -1;
    float step = 0.2;  // bin size for histogram
    vector<POINTCLOUD> pointCloudVec;
    POINTCLOUD pointCloudBuf;
    char* recvBuf = (char*)&pointCloudBuf;
    float* histBuf =
        (float*)malloc(sizeof(float) * int((vrMax - vrMin) / step));
    unsigned short curObjInd;
    struct timeval tv;
    struct tm tm;
    int cntPointCloud[2] = {0, 0};
    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm);
    char filePath[1024];
    unsigned char mode;
    sprintf(filePath, "data//%d_%d_%d_%d_%d_%d_altos.dat", tm.tm_year + 1900,
            tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    FILE* fp = fopen(filePath, "wb");

    long tmpTime = pointCloudBuf.pckHeader.sec;
    FILE* fp_time = fopen("timeVal.txt", "wt");
    while (rclcpp::ok()) {
        originPub->publish(origin);
        memset(recvBuf, 0, sizeof(POINTCLOUD));
        ret = recvfrom(sockfd, recvBuf, sizeof(POINTCLOUD), 0,
                       (struct sockaddr*)&from, &len);
        // printf("ret = %d\t%d\n", pointCloudBuf.pckHeader.objectCount, ret);
        if (ret > 0) {
            if ((pointCloudBuf.pckHeader.mode == 0 && cntPointCloud[1] > 0)) {
                if (static_cast<int>(pointCloudVec.size()) * POINTNUM <
                    cntPointCloud[0] + cntPointCloud[1]) {
                    printf(
                        "FrameId %d Loss %ld pack(s) in %d "
                        "packs------------------------\n",
                        pointCloudBuf.pckHeader.frameId,
                        int(ceil(cntPointCloud[0] / POINTNUM) +
                            ceil(cntPointCloud[1] / POINTNUM)) -
                            pointCloudVec.size(),
                        int(ceil(cntPointCloud[0] / POINTNUM) +
                            ceil(cntPointCloud[1] / POINTNUM)));
                }
                // cloud.width = pointCloudVec.size()*POINTNUM;
                // cloud.points.resize(cloud.width*cloud.height);
                for (size_t k = 0; k < widthSet * 2; k++) {
                    cloud.points[k].x =
                        0;  //(pointCloudVec[i].point[j].range)*cos(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);
                    cloud.points[k].y =
                        0;  //(pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);
                    cloud.points[k].z =
                        0;  //(pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);
                    cloud.points[k].h =
                        0;  // pointCloudVec[i].point[j].doppler;
                    cloud.points[k].s =
                        0;  // rcsCal(pointCloudVec[i].point[j].range,pointCloudVec[i].point[j].azi,pointCloudVec[i].point[j].snr,rcsBuf);
                }
                calPoint(pointCloudVec, cloud, installFlag, rcsBuf, step,
                         histBuf);
                std::chrono::duration<int, std::milli> s(5);
                rclcpp::sleep_for(s);
                pcl::toROSMsg(cloud, output);
                output.header.frame_id = "altosRadar";
                marker.header.frame_id = "altosRadar";
                marker.header.stamp = node->get_clock()->now();
                ostringstream str;
                str << "pointNum:" << cntPointCloud[0] + cntPointCloud[1];
                marker.text = str.str();
                marker.pose = pose;
                markerPub->publish(marker);
                output.header.stamp = node->get_clock()->now();
                pub->publish(output);
                pointCloudVec.clear();
                cntPointCloud[0] = 0;
                cntPointCloud[1] = 0;
                struct timeval tv;
                long long t1;
                gettimeofday(&tv, NULL);
                t1 = tv.tv_sec * 1000ll + tv.tv_usec / 1000;
                localtime_r(&tmpTime, &tm);
                fprintf(fp_time, "1:%f\n", t1 / 1e3);
            }
            fwrite(recvBuf, 1, ret, fp);
            curObjInd = pointCloudBuf.pckHeader.curObjInd;
            mode = pointCloudBuf.pckHeader.mode;
            cntPointCloud[mode] = pointCloudBuf.pckHeader.objectCount;
            pointCloudVec.push_back(pointCloudBuf);
            if ((mode == 1 && (static_cast<size_t>(curObjInd) + 1) * POINTNUM >=
                                  pointCloudBuf.pckHeader.objectCount)) {
                if (static_cast<int>(pointCloudVec.size()) * POINTNUM <
                    cntPointCloud[0] + cntPointCloud[1]) {
                    printf(
                        "FrameId %d %ld Loss %ld pack(s) in %d "
                        "packs------------------------\n",
                        pointCloudBuf.pckHeader.frameId, pointCloudVec.size(),
                        int(ceil(cntPointCloud[0] / POINTNUM) +
                            ceil(cntPointCloud[1] / POINTNUM)) -
                            pointCloudVec.size(),
                        int(ceil(cntPointCloud[0] / POINTNUM) +
                            ceil(cntPointCloud[1] / POINTNUM)));
                }
                // cloud.width = pointCloudVec.size()*POINTNUM;
                // cloud.points.resize(cloud.width*cloud.height);
                for (size_t k = 0; k < widthSet * 2; k++) {
                    cloud.points[k].x =
                        0;  //(pointCloudVec[i].point[j].range)*cos(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);
                    cloud.points[k].y =
                        0;  //(pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);
                    cloud.points[k].z =
                        0;  //(pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);
                    cloud.points[k].h =
                        0;  // pointCloudVec[i].point[j].doppler;
                    cloud.points[k].s =
                        0;  // rcsCal(pointCloudVec[i].point[j].range,pointCloudVec[i].point[j].azi,pointCloudVec[i].point[j].snr,rcsBuf);
                }

                calPoint(pointCloudVec, cloud, installFlag, rcsBuf, step,
                         histBuf);
                std::chrono::duration<int, std::milli> s(5);
                rclcpp::sleep_for(s);
                pcl::toROSMsg(cloud, output);
                output.header.frame_id = "altosRadar";
                marker.header.frame_id = "altosRadar";
                marker.header.stamp = node->get_clock()->now();
                ostringstream str;
                str << "pointNum:" << cntPointCloud[0] + cntPointCloud[1];
                printf("pointNum of %d frame: %d\n",
                       pointCloudBuf.pckHeader.frameId,
                       cntPointCloud[0] + cntPointCloud[1]);
                marker.text = str.str();
                marker.pose = pose;
                markerPub->publish(marker);
                output.header.stamp = node->get_clock()->now();
                pub->publish(output);
                pointCloudVec.clear();
                cntPointCloud[0] = 0;
                cntPointCloud[1] = 0;
                struct timeval tv;
                long long t1;
                gettimeofday(&tv, NULL);
                t1 = tv.tv_sec * 1000ll + tv.tv_usec / 1000;
                localtime_r(&tmpTime, &tm);
                fprintf(fp_time, "%f\n", t1 / 1e3);
                // cloud.clear();
            }
        } else {
            printf("recv failed (timeOut)   %d\n", ret);
        }
    }

    close(sockfd);
    free(histBuf);
    fclose(fp);
    fclose(fp_time);
    return 0;
}
