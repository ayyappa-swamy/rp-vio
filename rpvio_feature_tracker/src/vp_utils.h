#ifndef _VP_UTILS_H_
#define _VP_UTILS_H_
// # pragma once

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/line_descriptor.hpp>
#include <eigen3/Eigen/Dense>

#include "VPDetection.h"

using namespace std;
using namespace Eigen;
using namespace cv::line_descriptor;

/**
 * Draws the line segments on an image, coloured based on vanishing point directions
 **/
void drawClusters( cv::Mat &img, std::vector<KeyLine> &lines, std::vector<std::vector<int> > &clusters )
{
	// int cols = img.cols;
	// int rows = img.rows;

	//draw lines
	std::vector<cv::Scalar> lineColors( 3 );
	lineColors[0] = cv::Scalar( 0, 0, 255 );
	lineColors[1] = cv::Scalar( 0, 255, 0 );
	lineColors[2] = cv::Scalar( 255, 0, 0 );

	for ( size_t i=0; i<lines.size(); ++i )
	{
		int idx = (int)i;
		cv::Point pt_s = cv::Point( lines[idx].startPointX, lines[idx].startPointY);
		cv::Point pt_e = cv::Point( lines[idx].endPointX, lines[idx].endPointY);
		cv::Point pt_m = ( pt_s + pt_e ) * 0.5;

		cv::line( img, pt_s, pt_e, cv::Scalar(0,0,0), 2, CV_AA );
        // cv::putText( img, to_string(idx), pt_m, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));
	}

	for ( size_t i = 0; i < clusters.size(); ++i )
	{
		for ( size_t j = 0; j < clusters[i].size(); ++j )
		{
			int idx = clusters[i][j];

			cv::Point pt_s = cv::Point( lines[idx].startPointX, lines[idx].startPointY );
			cv::Point pt_e = cv::Point( lines[idx].endPointX, lines[idx].endPointY );
			// cv::Point pt_m = ( pt_s + pt_e ) * 0.5;

			cv::line( img, pt_s, pt_e, lineColors[i], 2, CV_AA );
		}
	}
}

int max_index(cv::Point3d point)
{
    vector<double> vec{fabs(point.x), fabs(point.y), fabs(point.z)};
    int max_id = 0;

    if (vec[1] > vec[max_id])
        max_id = 1;

    if (vec[2] > vec[max_id])
        max_id = 2;

    return max_id;
}

vector<int> align_vps(vector<cv::Point3d>& vps)
{
    vector<int> vp_ids(3);
    int idx0 = max_index(vps[0]);
    int idx1 = max_index(vps[1]);
    int idx2 = max_index(vps[2]);

    vp_ids[0] = idx0;
    vp_ids[1] = idx1;
    vp_ids[2] = idx2;

    return vp_ids;
}

void extract_lines_and_vps(
    cv::Mat &image, 
    std::vector<KeyLine> &lines_klsd, cv::Mat &lines_lsd_descr, 
    std::vector<cv::Point3d> &vps, std::vector<std::vector<int> > &clusters,
    std::vector<int> &lines_vps,
    double f, cv::Point2d pp, int LENGTH_THRESH = 0
)
{
    cv::Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor();
	// create an LSD detector
	cv::Ptr<LSDDetector> lsd = LSDDetector::createLSDDetector();
    
    // Detect lines using LSD
    vector<KeyLine> klsd;
	lsd->detect(image, klsd, 1, 1);
    
    for (size_t i = 0; i < klsd.size(); i++) {
		if (klsd[i].lineLength >= LENGTH_THRESH) {
			lines_klsd.push_back(klsd[i]);
		}
	}

	ROS_INFO("Detected %d line segments", (int)lines_klsd.size());

    // Compute binary descriptors
	bd->compute(image, lines_klsd, lines_lsd_descr);

    // Extract vps and clusters
	std::vector<cv::Point3d> vps_(3);              // Detected vanishing points
	std::vector<std::vector<int> > clusters_(3);   // Line segment clustering results of each vanishing point
	VPDetection detector;
	detector.run( lines_klsd, pp, f, vps_, clusters_ );

	ROS_INFO("Detected vanishing points are : ");
	for (size_t i = 0; i < vps_.size(); i++)
	{
	    ROS_INFO("%f, %f, %f", vps_[i].x, vps_[i].y, vps_[i].z);
	}
	
	std::vector<int> vp_ids = align_vps(vps_);
	vps[vp_ids[0]] = vps_[0];
	vps[vp_ids[1]] = vps_[1];
	vps[vp_ids[2]] = vps_[2];
	clusters[vp_ids[0]] = clusters_[0];
	clusters[vp_ids[1]] = clusters_[1];
	clusters[vp_ids[2]] = clusters_[2];
	

    // Map vp ids to lines
	lines_vps.resize(lines_klsd.size(), -1);
	for ( size_t i = 0; i < clusters.size(); ++i )
	{ 
		for ( size_t j = 0; j < clusters[i].size(); ++j )
		{
			int idx = clusters[i][j];
			lines_vps[idx] = (int)i;
		}
	}

	drawClusters(image, lines_klsd, clusters);
}

#endif // _VP_UTILS_H_