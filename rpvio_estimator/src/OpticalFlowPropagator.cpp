//
// Created by user on 2/9/22.
//

#include "OpticalFlowPropagator.h"

void OpticalFlowPropagator::reset(const ProcessedFrame &f) {
    this->source_frame = f;
}

cv::Mat OpticalFlowPropagator::propagate_farneback(const cv::Mat &rgb_img) {
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    return rgb_img;
}

OpticalFlowPropagator::OpticalFlowPropagator() {

}

OpticalFlowPropagator::~OpticalFlowPropagator() {

}

