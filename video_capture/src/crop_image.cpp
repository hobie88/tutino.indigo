#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <cvaux.h>


namespace bfs = boost::filesystem;

//input_dir where to look for pictures
bfs::path input_dir("/opt/ros/indigo/catkin_ws/src/video_capture/src/data2/");

CvRect detectFaceInImage(IplImage*,CvHaarClassifierCascade*);
IplImage* crop( IplImage*,  CvRect);
IplImage* resizeImage(const IplImage* ,int, int);


int main(int argc, char** argv)
{
    /**********
    bfs::create_directory(input_dir / "prova");
    std::ofstream file(input_dir / "prova/testo.txt");
    file << "ciao!";
    file.close();
    if (!bfs::exists(input_dir / "prova/testo.txt"))
        std::cout << "Something went wrong." << std::endl;
    else std::cout << "apposto" << std::endl;
    **************/
    static const char *faceCascadeFilename = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml";
    CvHaarClassifierCascade* faceCascade = (CvHaarClassifierCascade*)cvLoad(faceCascadeFilename, 0, 0, 0 );
     if( !faceCascade ) 
     {
       printf("Could not load Haar cascade Face detection classifier in '%s'.", faceCascadeFilename);
       exit(1);
     }   

    if (!bfs::exists(input_dir))
    { 
        std::cout << "Directory does not exist!" << std::endl;
        return -1; // check existence of input_dir
    }

    bfs::directory_iterator end_itr;
    //int pictures=0;
    std::vector<int> compression_params; //params for imwrite function
    compression_params.push_back(CV_IMWRITE_PXM_BINARY);
    compression_params.push_back(1);
    std::string filename;
    IplImage* img;
    //CvMat* mat;
    
    for (bfs::directory_iterator itr(input_dir); itr!=end_itr; itr++)
    {
        try
        { 
            filename = itr->path().string();
            img = cvLoadImage( filename.c_str(),1);
        }
        catch(int e)
        {
            std::cout << "An exception occured: exception n: " << e << std::endl;
        }
       CvRect rect = detectFaceInImage(img, faceCascade);
       std::cout << "3 cvrect ok" << std::endl;
       if (rect.x > 0 && rect.y > 0 && rect.height > 0 && rect.width > 0)
            img = crop(img,rect);
       
       //mat = cvCreateMat(img->height, img->width,CV_32FC3);
       //std::cout << "5 createMat ok" << std::endl;
       //cvConvert(img,mat);
       //std::cout << "6 convert ok" << std::endl;
       //cvNamedWindow( "check2", 1 );
       //cvShowImage( "check2", mat );
       //cv::waitKey(30);
       //std::string tmp;
       // std::cin>>tmp;
       if(cvSaveImage(filename.c_str(), img))
      // if (cv::imwrite(filename,(cv::InputArray)mat,compression_params))
           std::cout<<"image " << filename << " written" << std::endl;
       else
            std::cout<<"can't write image"<<std::endl;
      // std::cout << filename << std::endl;
        
    }
    

    
    return 0;
}

// Perform face detection on the input image, using the given Haar Cascade.
// Returns a rectangle for the detected region in the given image.
CvRect detectFaceInImage(IplImage *inputImg, CvHaarClassifierCascade* cascade)
{
	const CvSize minFeatureSize = cvSize(20, 20);
	const int flags = CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_DO_ROUGH_SEARCH;	// Only search for 1 face.
	const float search_scale_factor = 1.1f;
	IplImage *detectImg;
	IplImage *greyImg = 0;
	CvMemStorage* storage;
	CvRect rc;
	//double t;
	CvSeq* rects;


	storage = cvCreateMemStorage(0);
	cvClearMemStorage( storage );

	// If the image is color, use a greyscale copy of the image.
	detectImg = (IplImage*)inputImg;	// Assume the input image is to be used.
	if (inputImg->nChannels > 1)
	{
		greyImg = cvCreateImage(cvSize(inputImg->width, inputImg->height), IPL_DEPTH_8U, 1 );
		cvCvtColor( inputImg, greyImg, CV_BGR2GRAY );
		detectImg = greyImg;	// Use the greyscale version as the input.
	}

	// Detect all the faces.
	//$$$$$t = (double)cvGetTickCount();
	rects = cvHaarDetectObjects( detectImg, (CvHaarClassifierCascade*)cascade, storage,
				search_scale_factor, 3, flags, minFeatureSize );
	//$$$$$t = (double)cvGetTickCount() - t;
	//$$$$$ROS_INFO("[Face Detection took %d ms and found %d objects]\n", cvRound( t/((double)cvGetTickFrequency()*1000.0) ), rects->total );

	// Get the first detected face (the biggest).
	if (rects->total > 0)
        {
          rc = *(CvRect*)cvGetSeqElem( rects, 0 );
        }
	else
		rc = cvRect(-1,-1,-1,-1);	// Couldn't find the face.

	//cvReleaseHaarClassifierCascade( &cascade );
	//cvReleaseImage( &detectImg );
	if (greyImg)
	cvReleaseImage( &greyImg );
	cvReleaseMemStorage( &storage );

	return rc;	// Return the biggest face found, or (-1,-1,-1,-1).
}


IplImage* crop( IplImage* src,  CvRect roi)
{
    IplImage *imageTmp;
    IplImage * cropped;
    CvSize size;
	size.height = src->height;
	size.width = src->width;
  // Must have dimensions of output image
    imageTmp = cvCreateImage( size, IPL_DEPTH_8U, src->nChannels );
    cvCopy(src, imageTmp, NULL);
  // Say what the source region is
    cvSetImageROI( imageTmp, roi );

    size.width = roi.width;
	size.height = roi.height;
	cropped = cvCreateImage(size, IPL_DEPTH_8U, src->nChannels);
	cvCopy(imageTmp, cropped, NULL);	// Copy just the region.
    cropped = resizeImage(cropped,120,90);
    cvReleaseImage( &imageTmp );
	return cropped;
}


IplImage* resizeImage(const IplImage *origImg, int newWidth, int newHeight)
{
	IplImage *outImg = 0;
	int origWidth;
	int origHeight;
	if (origImg) {
		origWidth = origImg->width;
		origHeight = origImg->height;
	}
	if (newWidth <= 0 || newHeight <= 0 || origImg == 0 || origWidth <= 0 || origHeight <= 0) {
	    // marco edit ***********************************
	    std::cout << "foto: " << origImg->ID << std::endl;
	   //cvNamedWindow( "check2", 1 );
       //cvShowImage( "check2", origImg );
       //cv::waitKey(30);
       //std::string tmp;
       // std::cin>>tmp;

	    /**************************************************************************/
		printf("ERROR in resizeImage: Bad desired image size of %dx%d.", newWidth, newHeight);
		exit(1);
	}

	// Scale the image to the new dimensions, even if the aspect ratio will be changed.
	outImg = cvCreateImage(cvSize(newWidth, newHeight), origImg->depth, origImg->nChannels);
	if (newWidth > origImg->width && newHeight > origImg->height) {
		// Make the image larger
		cvResetImageROI((IplImage*)origImg);
		cvResize(origImg, outImg, CV_INTER_LINEAR);	// CV_INTER_CUBIC or CV_INTER_LINEAR is good for enlarging
	}
	else {
		// Make the image smaller
		cvResetImageROI((IplImage*)origImg);
		cvResize(origImg, outImg, CV_INTER_AREA);	// CV_INTER_AREA is good for shrinking / decimation, but bad at enlarging.
	}

	return outImg;
}
