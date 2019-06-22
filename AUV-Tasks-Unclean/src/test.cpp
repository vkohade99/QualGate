#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
using namespace std;
using namespace cv;
Mat src,src_gray,grad_x,abs_grad_x,src_blur;
int state_2_parameter=120;
int ddepth=CV_16S,scale=1,delta=0,h=3,s=0,v=0,hmax=14,smax=255,vmax=255,erosion_size=0,morph_size=1,size=10,line_parameter=15,min_line_length=40,max_line_gap=10,reflection_deleting_parameter=30;
vector<Vec4i> lines;
int state=0;
Point cent1,cent;
int main(int argc,char** argv)
{
  namedWindow("orig",WINDOW_FREERATIO);
  //namedWindow("gray",WINDOW_FREERATIO);
  namedWindow("edges",WINDOW_FREERATIO);
  namedWindow("Trackbars",WINDOW_FREERATIO);
  //namedWindow("blur",WINDOW_FREERATIO);
  	createTrackbar( "hue", "Trackbars", &h, 255, NULL);
  	createTrackbar( "sat", "Trackbars", &s, 255, NULL);
  	createTrackbar( "view", "Trackbars", &v, 255, NULL);
  	createTrackbar( "hue_max", "Trackbars", &hmax, 255, NULL);
  	createTrackbar( "sat_max", "Trackbars", &smax, 255, NULL);
  	createTrackbar( "view_max", "Trackbars", &vmax, 255, NULL);
    createTrackbar( "morphology", "Trackbars", &morph_size, 10, NULL);
    createTrackbar( "erosion", "Trackbars", &erosion_size, 10, NULL);
    createTrackbar( "blurkernel", "Trackbars", &size, 20, NULL);
    createTrackbar( "line parameter", "Trackbars", &line_parameter, 20, NULL);
    createTrackbar( "min line length", "Trackbars", &min_line_length, 200, NULL);
    createTrackbar( "max line gap", "Trackbars", &max_line_gap, 50, NULL);
    createTrackbar( "rd parameter", "Trackbars", &reflection_deleting_parameter, 80, NULL);
		createTrackbar( "State-2", "Trackbars", &state_2_parameter, 200, NULL);
  VideoCapture cap(argv[1]);
  while(cap.isOpened())
  {
		state=0;
    cap>>src;
    src(Range(0.50*src.rows,src.rows),Range::all());
    if(waitKey(10) == 'p')
			while(waitKey(10) == -1);
		if(src.empty())
			break;
    Mat element = getStructuringElement( 0,
                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       Point( erosion_size, erosion_size ) );
     Mat element1 = getStructuringElement( 0,
                        Size( 2*morph_size + 1, 2*morph_size+1 ),
                        Point( morph_size, morph_size ) );

    cvtColor( src, src_gray, CV_BGR2GRAY );
  //inRange(src,Scalar(h,s,v),Scalar(hmax,smax,vmax),src_gray);
  GaussianBlur( src, src_blur, Size( 2*size+1,2*size+1 ), 0, 0,BORDER_DEFAULT );
  Sobel( src_blur, grad_x, ddepth, 1, 0, 1, scale, delta, BORDER_DEFAULT );
  convertScaleAbs( grad_x, abs_grad_x );
  inRange(abs_grad_x,Scalar(h,s,v),Scalar(hmax,smax,vmax),abs_grad_x);
  erode(abs_grad_x,abs_grad_x,element);
  morphologyEx(abs_grad_x,abs_grad_x,MORPH_OPEN,element1);
  HoughLinesP(abs_grad_x, lines, 1, CV_PI/180, 150, min_line_length, max_line_gap );
  cout<<"\t\t"<<lines.size()<<endl;
	int a=-1;
	int b=-1,c=-1;
  int select_line=-1;
  static int xa0,xa1,xa2,xa3;
  static int xb0,xb1,xb2,xb3;
  for( size_t i = 0; i < lines.size(); i++ )
  {

    Vec4i l=lines[i];
    double angle = atan2 (abs(l[1]-l[3]),abs(l[0]-l[2])) * 180 / M_PI;
    //cout<<angle<<endl;
        if(abs(90.0-angle) < line_parameter && min(l[1],l[3])>120)//&&((l[1]>480-reflection_deleting_parameter)||(l[3]>480-reflection_deleting_parameter)))
				{
						if(a==-1)
						{
							a=i;
              xa0 = lines[a][0], xa1 = lines[a][1], xa2 = lines[a][2], xa3 = lines[a][3];
            }
						if(b==-1 && abs(xa0-l[0])>150 && (abs(xa1-xa3)-abs(l[1]-l[3]))<25)
						{
              int p,q;
              p=min(xa1,xa3);
              q=min(l[1],l[3]);
              if(abs(p-q)<30)
              {
                b=i;
                xb0 = lines[b][0], xb1 = lines[b][1], xb2 = lines[b][2], xb3 = lines[b][3];
                line( src, Point(xa0, xa1), Point(xa2, xa3), Scalar(0,0,255), 4, CV_AA);
                line( src, Point(xb0, xb1), Point(xb2, xb3), Scalar(0,255,0), 4, CV_AA);
                cent1.x=(xa0+xb0+xa2+xb2)/4;
								cent1.y=(xa1+xb1+xa3+xb3)/4;
                rectangle(src, Point (min(xa0,xa2),min(xa1,xa3)), Point (max(xb0,xb2),max(xb1,xb3)), Scalar(0,255,255),8,8,0);
                circle(src,cent1,10,Scalar(0,255,255),-1);
              }
						}
              if(c==-1)
              {
                int length_a=abs(xa1-xa3);
                if(b!=-1)
                {
                  int length_b=abs(xb1-xb3);
                  if(abs(l[0]-xb0)<0.43*abs(xa0-xb0) && abs(l[0]-xb0)>0.37*abs(xa0-xb0) && 0.35*length_b<abs(l[1]-l[3]) && abs(l[1]-l[3])<0.53*length_b)
                  {
                    int p,q;
                    p=min(xb1,xb3);
                    q=min(l[1],l[3]);
                    if(abs(p-q)<30)
                    {
                      c=i;
                      select_line=b;
                      line( src, Point(lines[c][0], lines[c][1]), Point(lines[c][2], lines[c][3]), Scalar(255,0,0), 4, CV_AA);
                      //line( src, Point(xb0, xb1), Point(xb2, xb3), Scalar(0,255,0), 4, CV_AA);
                      cent.x=(lines[select_line][0]+lines[c][0]+lines[select_line][2]+lines[c][2])/4;
                      cent.y=max(lines[c][1], lines[c][3]);
                      rectangle(src, Point (min(lines[c][0],lines[c][2]),min(lines[c][1],lines[c][3])), Point (max(xb0,xb2),max(xb1,xb3)), Scalar(255,0,0),8, 8, 0);
                      circle(src,cent,10,Scalar(255,0,0),-1);
                      break;
                    }
                  }
                  else if(abs(l[0]-xa0)<0.43*abs(xa0-xb0) && abs(l[0]-xa0)>0.37*abs(xa0-xb0) && 0.43*length_a<abs(l[1]-l[3]) && abs(l[1]-l[3])<0.53*length_a)
                  {
                    int p,q;
                    p=min(xa1,xa3);
                    q=min(l[1],l[3]);
                    if(abs(p-q)<15)
                    {
                      c=i;
                      select_line=a;
                      //line( src, Point(xa0, xa1), Point(xa2, xa3), Scalar(0,0,255), 4, CV_AA);
                      line( src, Point(lines[c][0], lines[c][1]), Point(lines[c][2], lines[c][3]), Scalar(255,0,0), 4, CV_AA);
                      cent.x=(lines[select_line][0]+lines[c][0]+lines[select_line][2]+lines[c][2])/4;
                      cent.y=max(lines[c][1], lines[c][3]);
                      circle(src,cent,10,Scalar(255,0,0),-1);
                      rectangle(src, Point (min(lines[c][0],lines[c][2]),min(lines[c][1],lines[c][3])), Point (max(xa0,xa2),max(xa1,xa3)), Scalar(255,0,0),8, 8, 0);
                      break;
                    }
                  }
                }
            }
					}

		}


    imshow("orig",src);
    //imshow("gray",src_gray);
    //imshow("blur",src_blur);
    imshow("edges",abs_grad_x);

  }

}
