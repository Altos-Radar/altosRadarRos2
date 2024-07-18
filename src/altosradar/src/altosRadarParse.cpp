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
#include <visualization_msgs/msg/marker.hpp>

using namespace std;
#define widthSet 4220
#define PORT 4040
#define vrMax 60
#define vrMin -60
#define errThr 1
#define PI 3.1415926

float hist(float* vr, float* histBuf, float step, int vrInd) {
    int bin_num = int((vrMax - vrMin) / step);

    for (int i = 0; i < vrInd; i++) {
        int ind = (vr[i] - vrMin) / step;

        if (vr[i] > 60 || vr[i] < -60 || isnan(vr[i])) {
            continue;
        }

        if (!(vr[i] > 0)) {
            histBuf[ind]++;
        }
    }

    return distance(histBuf, max_element(histBuf, histBuf + bin_num)) * step +
           vrMin;
}

float rcsCal(float range, float azi, float snr, float* rcsBuf) {
    int ind = (azi * 180 / PI + 60.1) * 10;
    float rcs = powf32(range, 2.6) * snr / 5.0e6 / rcsBuf[ind];
    return rcs;
}

int main(int argc, char** argv) {
    float* rcsBuf = (float*)malloc(1201 * sizeof(float));
    FILE* fp_rcs = fopen("data//rcs.dat", "rb");
    fread(rcsBuf, 1201, sizeof(float), fp_rcs);
    fclose(fp_rcs);

    rclcpp::init(argc, argv);
    rclcpp::Node node("altosRadar");
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
        node.create_publisher<sensor_msgs::msg::PointCloud2>("altosRadar", 10);
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub =
        node.create_publisher<visualization_msgs::msg::Marker>("points_number",
                                                               10);
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr originPub =
        node.create_publisher<visualization_msgs::msg::Marker>("origin", 10);

    visualization_msgs::msg::Marker origin;
    origin.header.frame_id = "altosRadar";
    origin.type = visualization_msgs::msg::Marker::SPHERE;
    origin.action = visualization_msgs::msg::Marker::ADD;

    origin.pose.position.x = 0;
    origin.pose.position.y = 0;
    origin.pose.position.z = 0;
    origin.pose.orientation.x = 0;
    origin.pose.orientation.y = 0;
    origin.pose.orientation.z = 0;
    origin.pose.orientation.w = 1;

    origin.scale.x = 1;
    origin.scale.y = 1;
    origin.scale.z = 1;
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
    marker.scale.z = 3;
    marker.color.b = 1.0f;
    marker.color.g = 1.0f;
    marker.color.r = 1.0f;
    marker.color.a = 1;
    geometry_msgs::msg::Pose pose;
    pose.position.x = (float)-5;
    pose.position.y = 0;
    pose.position.z = 0;

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

    int tmp;
    POINTCLOUD pointCloudBuf;
    char* recvBuf = (char*)&pointCloudBuf;
    struct timeval tv;
    struct tm tm;

    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm);
    char filePath[1024];
    sprintf(filePath, "data//%d_%d_%d_%d_%d_%d_altos.dat", tm.tm_year + 1900,
            tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    FILE* fp = fopen(filePath, "wb");
    int frameId = 0;
    int objectCntFrame = 0;
    int i;
    int objectCntLast, objectCnt;
    float offsetRange = 0;
    float offsetAzi = 0.0 * 3.1415926 / 180;
    float offsetEle = -0.0 * 3.1415926 / 180;
    float vr[widthSet];
    float vrAzi[widthSet];
    float step = 0.2;
    float* histBuf =
        (float*)malloc(sizeof(float) * int((vrMax - vrMin) / step));
    int vrInd = 0;
    float vrEst = 0;
    int cntFrameobj = 30;
    int installFlag = -1;
    unsigned char modeFlag = 0;
    long tmpTime = pointCloudBuf.pckHeader.sec;
    int cntPointCloud[2] = {0, 0};
    FILE* fp_time = fopen("timeVal.txt", "wt");
    while (rclcpp::ok()) {
        originPub->publish(origin);
        ret = recvfrom(sockfd, recvBuf, 1440, 0, (struct sockaddr*)&from, &len);
        if (ret > 0) {
            fwrite(recvBuf, 1, ret, fp);

            pointCloudBuf.pckHeader.curObjNum =
                pointCloudBuf.pckHeader.curObjNum / 44;
            objectCnt = pointCloudBuf.pckHeader.objectCount;
            pointCloudBuf.pckHeader.curObjInd =
                pointCloudBuf.pckHeader.curObjInd * 30;
            tmp = pointCloudBuf.pckHeader.frameId;
            modeFlag = tmp % 2;

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
                objectCntFrame = pointCloudBuf.pckHeader.curObjInd + POINTNUM;
                objectCntLast = objectCnt;
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
                vrEst = hist(vr, histBuf, step, vrInd);
                printf("Frame %d: objectCnt is %d\n", frameId, cntFrameobj);
                cntPointCloud[frameId % 2] = cntFrameobj;
                cntFrameobj = 0;
                objectCntLast = objectCnt;
                for (i = 0; i < vrInd; i++) {
                    cloud.points[i + widthSet * (frameId % 2)].v =
                        cloud.points[i + widthSet * (frameId % 2)].h -
                        vrEst * cos(vrAzi[i]);
                    if (cloud.points[i + widthSet * (frameId % 2)].v <
                        -errThr) {
                        cloud.points[i + widthSet * (frameId % 2)].v = -1;
                    } else if (cloud.points[i + widthSet * (frameId % 2)].v >
                               errThr) {
                        cloud.points[i + widthSet * (frameId % 2)].v = 1;
                    } else {
                        cloud.points[i + widthSet * (frameId % 2)].v = 0;
                    }
                }
                if (modeFlag == 1 || tmp - frameId == 2) {
                    std::chrono::duration<int, std::milli> s(5);
                    rclcpp::sleep_for(s);
                    pcl::toROSMsg(cloud, output);
                    rclcpp::Time now = node.get_clock()->now();

                    output.header.frame_id = "altosRadar";
                    marker.header.frame_id = "altosRadar";
                    marker.header.stamp = now;
                    ostringstream str;
                    str << "pointNum:" << cntPointCloud[0] + cntPointCloud[1];
                    marker.text = str.str();
                    marker.pose = pose;
                    markerPub->publish(marker);
                    output.header.stamp = now;
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
