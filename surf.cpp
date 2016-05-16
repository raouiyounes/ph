void  SURFExtractor(IplImage *frame){
 	 		  int minHessian = 400;
			  SurfFeatureDetector detector(minHessian);
	 
CvMat* image=0;

			   int Threshl=40;
			   int Octaves=2;
			   float PatternScales=1.0f;
image = cvCreateMat(frame->height, frame->width, CV_8UC1);
			  CvMemStorage* storage = cvCreateMemStorage(0);
			  cvCvtColor(frame, image, CV_BGR2GRAY);
			  //Define sequence for storing surf keypoints and descriptors
			  CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
			  int i,j;
		vector<double> descriptor_row;
			  CvSURFParams params = cvSURFParams(500, 1);
			  cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params);
			  MatrixXd descoo(imageDescriptors->total,5);
			  for( i = 0; i < imageDescriptors->total; i++ )
			  {
			double* r = (double*)cvGetSeqElem( imageDescriptors, i);
				j=0;
	while(j<5){
				cout<<*r;
				descoo(i,j)=*r;
			//	descriptor_row.push_back(*r);
			r++;
			j++;
			}
	vector<double> dd1;
	for(i=1;i<imageDescriptors->total;i++){
	for(int j=0;j<5;j++)
		dd1.push_back(descoo(i,j));
	number_of_line++;
	dd.push_back(descriptor_row);
	dd1.clear();
	}
	//descriptor_vect.push_back(descriptor_row);
	//number_of_line++;
		//	descriptor_row.clear();
			  }
		//	  return dd;

			  descooo=descoo;
}
