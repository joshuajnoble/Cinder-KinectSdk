#include "FaceTracker.h"

FaceTracker::FaceTracker( int rgbImageWidth, int rgbImageHeight, int depthWidth, int depthHeight, float zoom, int numUsers ) {

	// Video camera config with width, height, focal length in pixels
	// NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS focal length is computed for 640x480 resolution
	// If you use different resolutions, multiply this focal length by the scaling factor
	mVideoCameraConfig.Height = rgbImageHeight;
	mVideoCameraConfig.Width = rgbImageWidth;
	mVideoCameraConfig.FocalLength = NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS;

	// Depth camera config with width, height, focal length in pixels
	// NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS focal length is computed for 320x240 resolution
	// If you use different resolutions, multiply this focal length by the scaling factor
	mDepthCameraConfig.Height = depthHeight;
	mDepthCameraConfig.Width = depthWidth;
	mDepthCameraConfig.FocalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS;

	mZoomFactor = zoom;
	mNumUsers = numUsers;

	mContexts = new std::vector<FTHelperContext*>();

	context = new FTHelperContext();

	HRESULT hr;

	context->mFaceTracker = FTCreateFaceTracker();
	
	hr = context->mFaceTracker->Initialize( &mVideoCameraConfig, &mDepthCameraConfig, NULL, NULL );
	hr = context->mFaceTracker->CreateFTResult( &context->mFTResult );
	
				if( !SUCCEEDED(hr) ) {
				std::cout << " can't create mFTResult " << std::endl;
			}
			std::string s;
			switch ( hr ) {
				case E_POINTER:
					s = "E POINTER ";
					OutputDebugStringA(s.c_str());
					break;
				case S_OK:
					s = " OK ";
					OutputDebugStringA(s.c_str());
					break;
				case FT_ERROR_UNINITIALIZED:
					//trace( " FT_ERROR_UNINITIALIZED ");
					s = " FT_ERROR_UNINITIALIZED ";
					OutputDebugStringA(s.c_str());
					break;
				default:
					//trace( "Unknown error " );
					s = " unknown error ";
					OutputDebugStringA(s.c_str());
				}

	mNeedInitializeContexts = true;

}

//void FaceTracker::checkFaces(IFTImage* pColorImage, IFTImage* pDepthImage, int currentZoom, int viewOffset, int numberOfUsers) 
int FaceTracker::checkFaces(NUI_SKELETON_FRAME *skeleton, IFTImage *pColorImage, IFTImage *pDepthImage, int currentZoom, int viewOffset)
{

	if(mContexts->size() < mNumUsers) {
		// for each user we want to make a new face tracker
		for( int i = 0; i < mNumUsers; i++) {
			FTHelperContext *context = new FTHelperContext();
			//std::tr1::shared_ptr<FTHelperContext> context( new FTHelperContext() );
			//context->initialize( &mVideoCameraConfig, &mDepthCameraConfig );
			mContexts->push_back(context);
		}
	}

	HRESULT hr = S_OK;

// we just loop through all of them and update whichever ones can be tracked or are being tracked
	/*for (UINT i=0; i<mContexts->size(); i++)
    {

		if ( ( skeleton->SkeletonData + i )->eTrackingState != NUI_SKELETON_TRACKED && mContexts->at(i)->mLastTrackSucceeded) {

			mContexts->at(i)->mCountUntilFailure--;

			if(mContexts->at(i)->mCountUntilFailure == 0) {
					mContexts->at(i)->mLastTrackSucceeded = false;
			}

			continue; // bail
		}

		//NUI_SKELETON_DATA *skeletonData = skeleton;
		NUI_SKELETON_DATA *skeletonData = skeleton->SkeletonData + i;

        FT_VECTOR3D hint[2];

        hint[0].x = skeletonData->SkeletonPositions[NUI_SKELETON_POSITION_HEAD].x;
		hint[0].y = skeletonData->SkeletonPositions[NUI_SKELETON_POSITION_HEAD].y;
		hint[0].z = skeletonData->SkeletonPositions[NUI_SKELETON_POSITION_HEAD].z;

		hint[1].x = skeletonData->SkeletonPositions[NUI_SKELETON_POSITION_SPINE].x;
		hint[1].y = skeletonData->SkeletonPositions[NUI_SKELETON_POSITION_SPINE].y;
		hint[1].z = skeletonData->SkeletonPositions[NUI_SKELETON_POSITION_SPINE].z;

		FT_SENSOR_DATA sensorData(pColorImage, pDepthImage);

        if (mContexts->at(i)->mLastTrackSucceeded)
        {
            hr = mContexts->at(i)->mFaceTracker->ContinueTracking(&sensorData, hint, mContexts->at(i)->mFTResult);
        }
        else
        {
	        hr = mContexts->at(i)->mFaceTracker->StartTracking(&sensorData, NULL, hint, mContexts->at(i)->mFTResult);

			mContexts->at(i)->mLastTrackSucceeded = SUCCEEDED(hr) && SUCCEEDED(mContexts->at(i)->mFTResult->GetStatus());

			if(mContexts->at(i)->mLastTrackSucceeded ) {
				mContexts->at(i)->mSkeletonId = skeletonData->dwTrackingID;
				mContexts->at(i)->mCountUntilFailure = 30;
				
			} else {
				mContexts->at(i)->mFTResult->Reset();
			}
        }

        //mContexts->at(i)->mLastTrackSucceeded = SUCCEEDED(hrFT) && SUCCEEDED(m_UserContext[i].mFTResult->GetStatus());

		if(mContexts->at(i)->mLastTrackSucceeded == true) {
			// pass the handle to the face tracker
			std::cout << " getting a face " << std::endl;
			mContexts->at(i)->mFaceTracker->GetFaceModel( &mContexts->at(i)->mFTModel );
        }

        //SetCenterOfImage(m_UserContext[i].mFTResult);
    }*/

	NUI_SKELETON_DATA *skeletonData = skeleton->SkeletonData;

    FT_VECTOR3D hint[2];

    hint[0].x = skeletonData->SkeletonPositions[NUI_SKELETON_POSITION_HEAD].x;
	hint[0].y = skeletonData->SkeletonPositions[NUI_SKELETON_POSITION_HEAD].y;
	hint[0].z = skeletonData->SkeletonPositions[NUI_SKELETON_POSITION_HEAD].z;

	hint[1].x = skeletonData->SkeletonPositions[NUI_SKELETON_POSITION_SPINE].x;
	hint[1].y = skeletonData->SkeletonPositions[NUI_SKELETON_POSITION_SPINE].y;
	hint[1].z = skeletonData->SkeletonPositions[NUI_SKELETON_POSITION_SPINE].z;

	FT_SENSOR_DATA sensorData(pColorImage, pDepthImage);

        if (context->mLastTrackSucceeded)
        {
            hr =context->mFaceTracker->ContinueTracking(&sensorData, hint, context->mFTResult);
        }
        else
        {
	        hr = context->mFaceTracker->StartTracking(&sensorData, NULL, hint, context->mFTResult);

			context->mLastTrackSucceeded = SUCCEEDED(hr) && SUCCEEDED(context->mFTResult->GetStatus());

			if(context->mLastTrackSucceeded ) {
				context->mSkeletonId = skeletonData->dwTrackingID;
				context->mCountUntilFailure = 30;
				
			} else {
				context->mFTResult->Reset();
			}
        }

        //mContexts->at(i)->mLastTrackSucceeded = SUCCEEDED(hrFT) && SUCCEEDED(m_UserContext[i].mFTResult->GetStatus());

		if(context->mLastTrackSucceeded == true) {
			// pass the handle to the face tracker
			std::cout << " getting a face " << std::endl;
			context->mFaceTracker->GetFaceModel( &context->mFTModel );
        }

	return hr;
}

Vec2f FaceTracker::getCenterOfFace( UINT userId ) {

	Vec2f v;

	for( unsigned int i = 0; i < mContexts->size(); i++) {
		if(mContexts->at(i)->mSkeletonId == userId) {

			RECT faceCenter;
			mContexts->at(i)->mFTResult->GetFaceRect(&faceCenter);
			v.set( (float) faceCenter.left + (faceCenter.right - faceCenter.left / 2), (float) faceCenter.top + (faceCenter.top - faceCenter.bottom / 2));
			return v;
		}
	}  

	return v;
}

uint32_t FaceTracker::getNumFaces()
{
	int numFaces = 0;
	for( unsigned int i = 0; i < mContexts->size(); i++) {
		if(mContexts->at(i)->mCountUntilFailure > 0)
			numFaces++;
	}
	return numFaces;
}

std::vector<uint32_t> FaceTracker::getFaceTriangles( UINT userId ) 
{
	std::vector<uint32_t> vec;
	for( unsigned int i = 0; i < mContexts->size(); i++) {
		if(mContexts->at(i)->mSkeletonId == userId) {

			FT_TRIANGLE *faceIndices;
			UINT pPointCount;

			mContexts->at(i)->mFTModel->GetTriangles(&faceIndices, &pPointCount);

			for( int j = 0; j < (int) pPointCount; j++) {
				//Vec3f v(ppPoints[j].x, ppPoints[j].y );
				vec.push_back(faceIndices[j].i);
				vec.push_back(faceIndices[j].j);
				vec.push_back(faceIndices[j].k);
			}

			return vec;
		}
	}

	return vec; // just check for size() == 0;
}

void FaceTracker::getAnimationUnits( std::map<std::string, float> &units, UINT userId)
{
	for( unsigned int i = 0; i < mContexts->size(); i++) {
		if(mContexts->at(i)->mSkeletonId == userId) {
			for( unsigned int j = 0; j < mContexts->at(i)->mFTModel->GetAUCount(); j++) {
				//units[] = 
			}
			
		}
	}
}

int FaceTracker::getProjectedShape(UINT userId, float &scale, Vec3f &rotation, Vec3f &translation, std::vector<Vec2f> &projectedFacePoints )
{
	// catch all errors
	HRESULT hr = S_OK;

	for( unsigned int i = 0; i < mContexts->size(); i++) {
		if(mContexts->at(i)->mSkeletonId == userId && mContexts->at(i)->mLastTrackSucceeded) {

			UINT vertexCount = mContexts->at(i)->mFTModel->GetVertexCount();
			FT_VECTOR2D* pPts2D = reinterpret_cast<FT_VECTOR2D*>(_malloca(sizeof(FT_VECTOR2D) * vertexCount));

			// first get all the data about the face
			FLOAT fscale, rotationXYZ[3], translationXYZ[3];
			mContexts->at(i)->mFTResult->Get3DPose(&fscale, rotationXYZ, translationXYZ);

			// next animation units
			FLOAT *animationUnits;
			UINT auCount;
			hr = mContexts->at(i)->mFTResult->GetAUCoefficients(&animationUnits, &auCount);

			if(hr != S_OK)
				return hr;
			
			// now look at the shape units
			FLOAT *shapeUnitCoeffiecients;
			UINT shapeUnitCount;
			BOOL converged;
			hr = mContexts->at(i)->mFaceTracker->GetShapeUnits(&fscale, &shapeUnitCoeffiecients, &shapeUnitCount, &converged);

			if(hr != S_OK)
				return hr;

			// finally, actually get the projected 2d shape
			hr = mContexts->at(i)->mFTModel->GetProjectedShape(&mVideoCameraConfig, mZoomFactor, POINT(), shapeUnitCoeffiecients, mContexts->at(i)->mFTModel->GetSUCount(), animationUnits, auCount, 
                    fscale, rotationXYZ, translationXYZ, pPts2D, vertexCount);

			if(hr != S_OK)
				return hr;

			// now turn all the verts into something friendly
			for( int j = 0 ; j < vertexCount; j++) {
				projectedFacePoints.push_back(Vec2f( pPts2D[j].x, pPts2D[j].y));
			}

			translation.x = rotationXYZ[0];
			translation.y = rotationXYZ[1];
			translation.z = rotationXYZ[2];

			rotation.x = translationXYZ[0];
			rotation.y = translationXYZ[1];
			rotation.z = translationXYZ[2];
			
			scale = fscale;

			delete pPts2D;
		}
	}

	return hr;
}

ci::Rectf FaceTracker::getFaceRect( UINT userId )
{

	ci::Rectf r(0, 0, 100, 100);
	return r;
}

bool FaceTracker::lastTrackSucceeded()
{
	return mContexts->at(0)->mLastTrackSucceeded;
}
		
bool FaceTracker::lastTrackSucceeded(int userId)
{
	for( int i = 0; i < mContexts->size(); i++) {
		if(mContexts->at(i)->mSkeletonId == userId) {
			return mContexts->at(i)->mLastTrackSucceeded;
		}
	}
	return false;
}

void FaceTracker::startTracking( const FT_SENSOR_DATA *pSensorData, const RECT *pRoi, const FT_VECTOR3D headPoints[2])
{
	mContexts->at(0)->mFaceTracker->StartTracking(pSensorData, pRoi, headPoints, mContexts->at(0)->mFTResult);
}

void FaceTracker::continueTracking( const FT_SENSOR_DATA *pSensorData, const FT_VECTOR3D headPoints[2]) {
	mContexts->at(0)->mFaceTracker->ContinueTracking(pSensorData, headPoints, mContexts->at(0)->mFTResult);
}

void FaceTracker::startTracking( const FT_SENSOR_DATA *pSensorData, const RECT *pRoi, const FT_VECTOR3D headPoints[2], int skeletonId) {
	for( int i = 0; i < mContexts->size(); i++) {
		if(mContexts->at(i)->mSkeletonId == skeletonId)
			mContexts->at(0)->mFaceTracker->StartTracking(pSensorData, pRoi, headPoints, mContexts->at(0)->mFTResult);
	}
}
void FaceTracker::continueTracking( const FT_SENSOR_DATA *pSensorData, const FT_VECTOR3D headPoints[2], int skeletonId)
{
		for( int i = 0; i < mContexts->size(); i++) {
		if(mContexts->at(i)->mSkeletonId == skeletonId)
			mContexts->at(0)->mFaceTracker->ContinueTracking(pSensorData, headPoints, mContexts->at(0)->mFTResult);
	}
}
