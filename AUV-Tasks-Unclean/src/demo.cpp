#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using std::cout;

int main( int argc, char** argv )
{
    String imageName; // by default
    if (argc > 1)
    {
        imageName = argv[1];
    }
    Mat src = imread( imageName, IMREAD_COLOR ); // Load an image
    if (src.empty())
    {
        cout << "Cannot read image: " << imageName << std::endl;
        return -1;
    }
    imshow("123",src);
    waitKey();
    return 0;
}
