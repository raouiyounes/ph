int Image::SURFExtractor(IplImage* frame,int nb_of_features){
	//  Mat img_1 = image_data;
	 	  //-- Step 1: Detect the keypoints using SURF Detector
	  int minHessian = 400;
	  SurfFeatureDetector detector( minHessian );
	 	   int Threshl=40;
	   int Octaves=2;
	   float PatternScales=1.0f;
	  CvMat* prevgray = 0, *image = 0, *gray =0;
//IplImage* frame=cvLoadImage("test_image.jpg");
image = cvCreateMat(frame->height, frame->width, CV_8UC1);
	  CvMemStorage* storage = cvCreateMemStorage(0);
	  cvCvtColor(frame, image, CV_BGR2GRAY);
	  //Define sequence for storing surf keypoints and descriptors
	  CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
	  int i,j;
	  ofstream file("surf.txt");
vector<double> descriptor_row;
	  CvSURFParams params = cvSURFParams(500, 1);
	  cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params );
	  for( i = 0; i < imageDescriptors->total; i++ )
	  {
	float* r = (float*)cvGetSeqElem( imageDescriptors, i);
	j=0;
int dim=50;
	// changer dim pour obtenir plus de features
	while(j<nb_of_features){
descriptor_row.push_back(*r);
	r++;
	j++;
	file<<*r<<" ";
	}
	file<<"\n";
	descriptor_vect.push_back(descriptor_row);
	number_of_line++;
	descriptor_row.clear();
	  }
	  return 0;
}
