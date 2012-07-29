#include "FaceTracker.h"

FaceTracker::FaceTracker( int rgbImageHeight, int rgbImageWidth, int depthWidth, int depthHeight, int numUsers ) {

	// Video camera config with width, height, focal length in pixels
	// NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS focal length is computed for 640x480 resolution
	// If you use different resolutions, multiply this focal length by the scaling factor
	FT_CAMERA_CONFIG videoCameraConfig = {640, 480, NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS};

	// Depth camera config with width, height, focal length in pixels
	// NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS focal length is computed for 320x240 resolution
	// If you use different resolutions, multiply this focal length by the scaling factor
	FT_CAMERA_CONFIG depthCameraConfig = {320, 240, NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS};

	// for each user we want to make a new face tracker
	//for( int i = 0; i < kinect->getUserCount(); i++) {
	for( int i = 0; i < numUsers; i++) {
		FTHelperContext *context = new FTHelperContext();
		context->m_pFaceTracker = FTCreateFaceTracker(NULL);
		context->m_pFaceTracker->Initialize( &videoCameraConfig, &depthCameraConfig, NULL, NULL );
		mContexts.push_back(context);
	}

}

//void FaceTracker::checkFaces(IFTImage* pColorImage, IFTImage* pDepthImage, int currentZoom, int viewOffset, int numberOfUsers) 
void FaceTracker::checkFaces(NUI_SKELETON_FRAME *skeleton, IFTImage *pColorImage, IFTImage *pDepthImage, int currentZoom, int viewOffset)
{
// we just loop through all of them and update whichever ones can be tracked or are being tracked
    for (UINT i=0; i<NUI_SKELETON_COUNT; i++)
    {

		if ( ( skeleton->SkeletonData + i )->eTrackingState != NUI_SKELETON_TRACKED && mContexts[i]->m_LastTrackSucceeded) {

			mContexts[i]->m_CountUntilFailure--;

			if(mContexts[i]->m_CountUntilFailure == 0) {
					mContexts[i]->m_LastTrackSucceeded = false;
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

		HRESULT hrFT = E_FAIL;

		FT_SENSOR_DATA sensorData(pColorImage, pDepthImage);

        if (mContexts[i]->m_LastTrackSucceeded)
        {
            hrFT = mContexts[i]->m_pFaceTracker->ContinueTracking(&sensorData, hint, mContexts[i]->m_pFTResult);
        }
        else
        {
				mContexts[i]->m_LastTrackSucceeded = true;
				mContexts[i]->m_SkeletonId = skeletonData->dwTrackingID;
				mContexts[i]->m_CountUntilFailure = 30;
	            hrFT = mContexts[i]->m_pFaceTracker->StartTracking(&sensorData, NULL, hint, mContexts[i]->m_pFTResult);
        }

        //mContexts[i]->m_LastTrackSucceeded = SUCCEEDED(hrFT) && SUCCEEDED(m_UserContext[i].m_pFTResult->GetStatus());
		if (mContexts[i]->m_pFTResult->GetStatus() == 1 && mContexts[i]->m_LastTrackSucceeded)
        {
			// pass the handle to the face tracker
			mContexts[i]->m_pFaceTracker->GetFaceModel( &mContexts[i]->m_pFTModel );
        }

        //SetCenterOfImage(m_UserContext[i].m_pFTResult);
    }
}

Vec2f FaceTracker::getCenterOfFace( UINT userId ) {

	Vec2f v;

	for( unsigned int i = 0; i < mContexts.size(); i++) {
		if(mContexts[i]->m_SkeletonId == userId) {

			RECT faceCenter;
			mContexts[i]->m_pFTResult->GetFaceRect(&faceCenter);
			v.set( (float) faceCenter.left + (faceCenter.right - faceCenter.left / 2), (float) faceCenter.top + (faceCenter.top - faceCenter.bottom / 2));
			return v;
		}
	}  

	return v;
}

std::vector<Vec2f> FaceTracker::getFacePoints( UINT userId )
{
	std::vector<Vec2f> vec;
	for( unsigned int i = 0; i < mContexts.size(); i++) {
		if(mContexts[i]->m_SkeletonId == userId) {

			FLOAT scale;
			FLOAT rotationXYZ[3];
			FLOAT translationXYZ[3];

			FT_VECTOR2D *ppPoints;
			UINT *pPointCount;

			mContexts[i]->m_pFTResult->Get2DShapePoints(&ppPoints, pPointCount);

			for( int j = 0; j < (int) pPointCount; j++) {
				Vec2f v(ppPoints[j].x, ppPoints[j].y );
				vec.push_back(v);
			}

			return vec;
		}
	}

	return vec; // just check for size() == 0;
}

int FaceTracker::get3DPose( UINT userId, float *scale, Vec3f *rotation, Vec3f *translation )
{

	for( unsigned int i = 0; i < mContexts.size(); i++) {
		if(mContexts[i]->m_SkeletonId == userId) {

			FLOAT scale;
			FLOAT rotationXYZ[3];
			FLOAT translationXYZ[3];

			mContexts[i]->m_pFTResult->Get3DPose(&scale, &rotationXYZ[0], &translationXYZ[0]);

			rotation->set( rotationXYZ[0], rotationXYZ[1], rotationXYZ[2] );
			translation->set( translationXYZ[0], translationXYZ[1], translationXYZ[2] );
			return 1;
		}
	} 
	return 0;
}

ci::Rectf FaceTracker::getFaceRect( UINT userId )
{

	ci::Rectf r(0, 0, 100, 100);
	return r;
}

bool FaceTracker::lastTrackSucceeded()
{
	return mContexts[0]->m_LastTrackSucceeded;
}
		
bool FaceTracker::lastTrackSucceeded(int userId)
{
	for( int i = 0; i < mContexts.size(); i++) {
		if(mContexts[i]->m_SkeletonId == userId) {
			return mContexts[i]->m_LastTrackSucceeded;
		}
	}
	return false;
}

void FaceTracker::startTracking( const FT_SENSOR_DATA *pSensorData, const RECT *pRoi, const FT_VECTOR3D headPoints[2])
{
	mContexts[0]->m_pFaceTracker->StartTracking(pSensorData, pRoi, headPoints, mContexts[0]->m_pFTResult);
}

void FaceTracker::continueTracking( const FT_SENSOR_DATA *pSensorData, const FT_VECTOR3D headPoints[2]) {
	mContexts[0]->m_pFaceTracker->ContinueTracking(pSensorData, headPoints, mContexts[0]->m_pFTResult);
}

void FaceTracker::startTracking( const FT_SENSOR_DATA *pSensorData, const RECT *pRoi, const FT_VECTOR3D headPoints[2], int skeletonId) {
	for( int i = 0; i < mContexts.size(); i++) {
		if(mContexts[i]->m_SkeletonId == skeletonId)
			mContexts[0]->m_pFaceTracker->StartTracking(pSensorData, pRoi, headPoints, mContexts[0]->m_pFTResult);
	}
}
void FaceTracker::continueTracking( const FT_SENSOR_DATA *pSensorData, const FT_VECTOR3D headPoints[2], int skeletonId)
{
		for( int i = 0; i < mContexts.size(); i++) {
		if(mContexts[i]->m_SkeletonId == skeletonId)
			mContexts[0]->m_pFaceTracker->ContinueTracking(pSensorData, headPoints, mContexts[0]->m_pFTResult);
	}
}
