/*
 * @Copyright (C) 2024 by Altos Radar. All rights reserved.
 * @Author: altos
 * @Description: Altos Radar Driver
 */

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "altosradar/PointCloud.h"

using namespace std;
#define widthSet 16000
#define PORT 4040
#define vrMax 60
#define vrMin -60
#define errThr 3
#define PI 3.1415926
#define RECVSIZEMAX 10000

/// @brief Calculate the radar cross section(rcs) of single point
/// @return rcs
float rcsCal(float range, float azi, float snr, float *rcsBuf) {
    int ind = (azi * 180 / PI + 60.1) * 10;
    float rcs = powf32(range, 2.6) * snr / 5.0e6 / rcsBuf[ind];
    return rcs;
}

/// @brief Calculate the radar ego velocity based on histogram. The assumption
/// is that the statc points are more than the moving points.
/// @return velocity
float hist(float *vr, float *histBuf, float step, int vrInd) {
    int ind = 0;
    for (int i = 0; i < vrInd; i++) {
        ind = (vr[i] - vrMin) / step;
        // printf("ind = %d\n",i);

        if (vr[i] > 60 || vr[i] < -60 || isnan(vr[i])) {
            // printf("vr[%d] = %f\n",ind,vr[i]);
            continue;
        }
        if (vr[i] <= 0 && vr[i] > -40) {
            histBuf[ind]++;
        }
    }
    return float(
               (max_element(histBuf, histBuf + (int((vrMax - vrMin) / step))) -
                histBuf)) *
               step +
           vrMin;
}

int main(int argc, char **argv) {
    float *rcsBuf = (float *)malloc(1201 * sizeof(float));
    FILE *fp_rcs = fopen("data//rcs.dat", "rb");
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
    printf("---------------------------\n");
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
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
               sizeof(struct timeval));
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
               sizeof(struct timeval));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    int ret = bind(sockfd, (struct sockaddr *)&addr, sizeof(addr));
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

    int tmp;
    POINTCLOUD pointCloudBuf;
    char *recvPoint = (char *)&pointCloudBuf;
    char *recvBuf = (char *)malloc(RECVSIZEMAX);
    struct timeval tv;
    struct tm tm;

    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm);
    char filePath[1024];
    sprintf(filePath, "data//%d_%d_%d_%d_%d_%d_altos.dat", tm.tm_year + 1900,
            tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    FILE *fp_point = fopen(filePath, "wb");
    sprintf(filePath, "data//%d_%d_%d_%d_%d_%d_altosRD.dat", tm.tm_year + 1900,
            tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

    int frameId = 0;
    int i;
    int objectCntLast, objectCnt;
    float offsetRange = 0;
    float offsetAzi = 0.0 * 3.1415926 / 180;
    float offsetEle = -0.0 * 3.1415926 / 180;
    float vr[widthSet];
    float vrAzi[widthSet];
    float step = 0.2;
    float *histBuf =
        (float *)malloc(sizeof(float) * int((vrMax - vrMin) / step));

    int vrInd = 0;
    float vrEst = 0;
    int cntFrameobj = 30;
    int installFlag = -1;
    unsigned char modeFlag = 0;
    long tmpTime = pointCloudBuf.pckHeader.sec;
    int cntPointCloud[2] = {0, 0};
    FILE *fp_time = fopen("timeVal.txt", "wt");
    FILE *fp_terminal = fopen("terminal.txt", "wt");

    while (rclcpp::ok()) {
        originPub->publish(origin);
        ret = recvfrom(sockfd, recvBuf, RECVSIZEMAX, 0,
                       (struct sockaddr *)&from, &len);
        // printf("ret = %d\n",ret);
        if (ret > 0) {
            if (ret <= 1368) {
                memcpy(recvPoint, recvBuf, ret);
                // fwrite(recvPoint, 1, ret, fp_point);
            }

            // printf("%d\tpointCloudBuf.pckHeader.objectCount = %d
            // \tpckHeader.curObjNum =
            // %d\n",pointCloudBuf.pckHeader.frameId,pointCloudBuf.pckHeader.curObjInd,pointCloudBuf.pckHeader.curObjNum/44);
            // long tmpTime = pointCloudBuf.pckHeader.sec;
            // localtime_r(&tmpTime, &tm);
            // printf("%d_%d_%d_%d_%d_%d\n",tm.tm_year + 1900,tm.tm_mon +
            // 1,tm.tm_mday,tm.tm_hour,tm.tm_min,pointCloudBuf.pckHeader.sec);
            pointCloudBuf.pckHeader.curObjNum =
                pointCloudBuf.pckHeader.curObjNum / 44;
            objectCnt = pointCloudBuf.pckHeader.objectCount;
            pointCloudBuf.pckHeader.curObjInd =
                pointCloudBuf.pckHeader.curObjInd * 30;
            tmp = pointCloudBuf.pckHeader.frameId;
            modeFlag = 0;
            // if(pointCloudBuf.pckHeader.mode==1)
            // {
            //     continue;
            // }
            if (frameId == 0 || frameId == tmp) {
                frameId = tmp;
                for (i = 0; i < pointCloudBuf.pckHeader.curObjNum; i = i + 1) {
                    if (abs(pointCloudBuf.point[i].range) > 0) {
                        pointCloudBuf.point[i].ele =
                            installFlag *
                            (pointCloudBuf.point[i].ele - offsetEle);
                        pointCloudBuf.point[i].azi =
                            -installFlag *
                            asin(sin(pointCloudBuf.point[i].azi) /
                                 cos(pointCloudBuf.point[i].ele));
                        cloud
                            .points[pointCloudBuf.pckHeader.curObjInd + i +
                                    widthSet * modeFlag]
                            .x = (pointCloudBuf.point[i].range - offsetRange) *
                                 cos(pointCloudBuf.point[i].azi - offsetAzi) *
                                 cos(pointCloudBuf.point[i].ele);
                        cloud
                            .points[pointCloudBuf.pckHeader.curObjInd + i +
                                    widthSet * modeFlag]
                            .y = (pointCloudBuf.point[i].range - offsetRange) *
                                 sin(pointCloudBuf.point[i].azi - offsetAzi) *
                                 cos(pointCloudBuf.point[i].ele);
                        cloud
                            .points[pointCloudBuf.pckHeader.curObjInd + i +
                                    widthSet * modeFlag]
                            .z = (pointCloudBuf.point[i].range - offsetRange) *
                                 sin(pointCloudBuf.point[i].ele);
                        cloud
                            .points[pointCloudBuf.pckHeader.curObjInd + i +
                                    widthSet * modeFlag]
                            .h = pointCloudBuf.point[i].doppler;
                        cloud
                            .points[pointCloudBuf.pckHeader.curObjInd + i +
                                    widthSet * modeFlag]
                            .s = rcsCal(pointCloudBuf.point[i].range,
                                        pointCloudBuf.point[i].azi,
                                        pointCloudBuf.point[i].snr, rcsBuf);
                        vr[pointCloudBuf.pckHeader.curObjInd + i] =
                            pointCloudBuf.point[i].doppler /
                            cos(pointCloudBuf.point[i].azi - offsetAzi);

                        vrAzi[pointCloudBuf.pckHeader.curObjInd + i] =
                            pointCloudBuf.point[i].azi;
                        cntFrameobj++;
                    }
                }
                vrInd = pointCloudBuf.pckHeader.curObjInd + POINTNUM;
                // objectCntFrame = pointCloudBuf.pckHeader.curObjInd +
                // POINTNUM;
                objectCntLast = objectCnt;
                // cntFrameobj = cntFrameobj + 30;

            } else {
                if (objectCntLast - cntFrameobj > POINTNUM) {
                    printf(
                        "-------------------------dataLoss %d\t%d\t%d pack(s) "
                        "in %d packs------------------------\n",
                        cntFrameobj, objectCntLast,
                        (objectCntLast - cntFrameobj) / POINTNUM,
                        objectCntLast / POINTNUM);
                }
                memset(histBuf, 0, sizeof(float) * int((vrMax - vrMin) / step));
                printf("Frame %d: objectCnt is %d\n", frameId, objectCntLast);
                vrEst = hist(vr, histBuf, step, vrInd);
                // printf("Frame %d: objectCnt is %d\n",frameId,cntFrameobj);
                cntPointCloud[0] = cntFrameobj;
                cntFrameobj = 0;
                objectCntLast = objectCnt;
                for (i = 0; i < vrInd; i++) {
                    cloud.points[i + widthSet * (frameId % 2) * 0].v =
                        cloud.points[i + widthSet * (frameId % 2) * 0].h -
                        vrEst * cos(vrAzi[i]);
                    if (cloud.points[i + widthSet * (frameId % 2) * 0].v <
                        -errThr) {
                        cloud.points[i + widthSet * (frameId % 2) * 0].v = -1;
                    } else if (cloud.points[i + widthSet * (frameId % 2) * 0]
                                   .v > errThr) {
                        cloud.points[i + widthSet * (frameId % 2) * 0].v = 1;
                    } else {
                        cloud.points[i + widthSet * (frameId % 2) * 0].v = 0;
                    }
                }
                if (modeFlag == 0) {
                    std::chrono::duration<int, std::milli> s(5);
                    rclcpp::sleep_for(s);
                    pcl::toROSMsg(cloud, output);
                    output.header.frame_id = "altosRadar";
                    marker.header.frame_id = "altosRadar";
                    marker.header.stamp = node->get_clock()->now();
                    ostringstream str;
                    str << "pointNum:" << cntPointCloud[0];
                    marker.text = str.str();
                    marker.pose = pose;
                    markerPub->publish(marker);
                    output.header.stamp = node->get_clock()->now();
                    pub->publish(output);
                    for (int i = 0; i < widthSet * 2; i++) {
                        cloud.points[i].x = 0;
                        cloud.points[i].y = 0;
                        cloud.points[i].z = 0;
                        cloud.points[i].h = 0;
                        cloud.points[i].s = 0;
                        cloud.points[i].v = 0;
                    }
                    struct timeval tv;

                    long long t1;

                    gettimeofday(&tv, NULL);

                    t1 = tv.tv_sec * 1000ll + tv.tv_usec / 1000;
                    localtime_r(&tmpTime, &tm);
                    fprintf(fp_time, "%f\n", t1 / 1e3);
                }
                frameId = tmp;
                for (int i = 0; i < pointCloudBuf.pckHeader.curObjNum; i++) {
                    if (abs(pointCloudBuf.point[i].range) > 0) {
                        pointCloudBuf.point[i].ele =
                            installFlag *
                            (pointCloudBuf.point[i].ele - offsetEle);
                        pointCloudBuf.point[i].azi =
                            -installFlag *
                            asin(sin(pointCloudBuf.point[i].azi) /
                                 cos(pointCloudBuf.point[i].ele));
                        cloud
                            .points[pointCloudBuf.pckHeader.curObjInd + i +
                                    widthSet * modeFlag]
                            .x = (pointCloudBuf.point[i].range - offsetRange) *
                                 cos(pointCloudBuf.point[i].azi - offsetAzi);
                        cloud
                            .points[pointCloudBuf.pckHeader.curObjInd + i +
                                    widthSet * modeFlag]
                            .y = (pointCloudBuf.point[i].range - offsetRange) *
                                 sin(pointCloudBuf.point[i].azi - offsetAzi);
                        cloud
                            .points[pointCloudBuf.pckHeader.curObjInd + i +
                                    widthSet * modeFlag]
                            .z = (pointCloudBuf.point[i].range - offsetRange) *
                                 sin(pointCloudBuf.point[i].ele);
                        cloud
                            .points[pointCloudBuf.pckHeader.curObjInd + i +
                                    widthSet * modeFlag]
                            .h = pointCloudBuf.point[i].doppler;
                        cloud
                            .points[pointCloudBuf.pckHeader.curObjInd + i +
                                    widthSet * modeFlag]
                            .s = rcsCal(pointCloudBuf.point[i].range,
                                        pointCloudBuf.point[i].azi,
                                        pointCloudBuf.point[i].snr, rcsBuf);
                        vr[pointCloudBuf.pckHeader.curObjInd + i] =
                            pointCloudBuf.point[i].doppler /
                            cos(pointCloudBuf.point[i].azi - offsetAzi);
                        vrAzi[pointCloudBuf.pckHeader.curObjInd + i] =
                            pointCloudBuf.point[i].azi;
                        cntFrameobj++;
                    }
                }
                vrInd = pointCloudBuf.pckHeader.curObjInd + POINTNUM;
            }
        } else {
            fprintf(fp_terminal, "recv failed (timeOut)   %d\n", ret);
        }
    }
    close(sockfd);
    free(histBuf);
    free(recvBuf);
    fclose(fp_point);
    fclose(fp_time);
    fclose(fp_terminal);

    return 0;
}
