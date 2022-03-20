#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

#ifdef HAVE_OPENCV_FEATURES2D

#include <opencv2/line_descriptor.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

using namespace std;
using namespace cv;
using namespace cv::line_descriptor;

int main()
{   
    ifstream lines1("im1_vp_lines.txt", ios_base::in);
    ifstream lines2("im2_vp_lines.txt", ios_base::in);
    
    vector<KeyLine> kls1;
    vector<KeyLine> kls2;

    // Read the images
    cv::Mat im1 = imread("view1_masked_facade5.png", 0);
    cv::Mat im2 = imread("view2_masked_facade5.png", 0);

    string row_string;

    // Read the lines
    while(getline(lines1, row_string))
    {   
        stringstream row_stream(row_string);

        float startX, endX, startY, endY;
        row_stream >> startX >> endX >> startY >> endY;

        KeyLine kl = KeyLine();
        kl.startPointX = startX;
        kl.startPointY = startY;
        kl.endPointX = endX;
        kl.endPointY = endY;

        kls1.push_back(kl);
    }

    while(getline(lines2, row_string))
    {   
        stringstream row_stream(row_string);

        float startX, endX, startY, endY;
        row_stream >> startX >> endX >> startY >> endY;

        KeyLine kl = KeyLine();
        kl.startPointX = startX;
        kl.startPointY = startY;
        kl.endPointX = endX;
        kl.endPointY = endY;

        kls2.push_back(kl);
    }

    // Create Binary Descriptor Extractors
    Mat descr1, descr2;

    /* create a pointer to a BinaryDescriptor object with default parameters */
    Ptr<BinaryDescriptor> bd1 = BinaryDescriptor::createBinaryDescriptor();
    bd1->compute(im1, kls1, descr1);

    Ptr<BinaryDescriptor> bd2 = BinaryDescriptor::createBinaryDescriptor();
    bd2->compute(im2, kls2, descr2);

    cout << "Number of lines in view2 " << kls1.size() << endl;
    cout << "Number of descriptors in view2 " << descr2.rows << endl;

    /* create a BinaryDescriptorMatcher object */
    Ptr<BinaryDescriptorMatcher> bdm = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
    
    // Draw the line matches and display
    /* compute matches */
    std::vector<DMatch> lsd_matches;
    bdm->match(descr1, descr2, lsd_matches);

    cout << "Number of descr matches " << lsd_matches.size() << endl;

    // /* select best matches */
    // std::vector<DMatch> good_matches;
    // for (int j = 0; j < (int)lsd_matches.size(); j++)
    // {
    // // if ((lsd_matches[j].distance < 30))
    //     good_matches.push_back(lsd_matches[j]);
    // }

    /* plot matches */
    Mat lsd_outImg;

    std::vector<char> lsd_mask(lsd_matches.size(), 1);
    drawLineMatches(im1, kls1, im2, kls2, lsd_matches, lsd_outImg, Scalar::all(-1), Scalar::all(-1), lsd_mask,
                    DrawLinesMatchesFlags::DEFAULT);

    imwrite("im1_im2_vp_matches.png", lsd_outImg);

    return 0;
}

#else

int main()
{
  std::cerr << "OpenCV was built without features2d module" << std::endl;
  return 0;
}

#endif // HAVE_OPENCV_FEATURES2D