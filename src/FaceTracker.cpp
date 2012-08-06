#include "FaceTracker.h"

void FTHelperContext::initialize( FT_CAMERA_CONFIG *videoCameraConfig, FT_CAMERA_CONFIG *depthCameraConfig) 
{

	mFaceTracker = FTCreateFaceTracker();

	if(!mFaceTracker) {
		trace( "  FTHelperContext::initialize Can't create face tracker " );
	}

	HRESULT hr;

	hr = mFaceTracker->Initialize( videoCameraConfig, depthCameraConfig, NULL, NULL );
	//E_INVALIDARG, E_POINTER, FT_ERROR_INVALID_MODEL_PATH, FT_ERROR_INVALID_MODELS, E_OUTOFMEMORY
	switch ( hr ) {
		case E_POINTER:
			trace( "Bad pointer." );
			break;
			case FT_ERROR_INVALID_MODEL_PATH:
			trace( " FT_ERROR_INVALID_MODEL_PATH " );
			break;
			case FT_ERROR_INVALID_MODELS:
			trace( " FT_ERROR_INVALID_MODELS " );
			break;
			case E_OUTOFMEMORY:
			trace( " E_OUTOFMEMORY " );
			break;
							case E_INVALIDARG:
			trace( " E_INVALIDARG " );
			break;
		case S_OK:
			break;
		case FT_ERROR_UNINITIALIZED:
			trace( " FT_ERROR_UNINITIALIZED ");
			break;
		case FT_ERROR_KINECT_DLL_FAILED:
			trace(" dll failed ");
			break;
		case S_FALSE:
			trace( "Data not available." );
			break;
				default:
					std::stringstream ss;
					ss << hr;
					trace( "Unknown error " );
					trace(ss.str());
					break;
		}

	hr = mFaceTracker->CreateFTResult(&mFTResult);

	if( !SUCCEEDED(hr) ) {
		std::cout << " can't create mFTResult " << std::endl;
	}

	switch ( hr ) {
		case E_POINTER:
			trace( "Bad pointer." );
			break;
		case S_OK:
			break;
		case FT_ERROR_UNINITIALIZED:
			trace( " FT_ERROR_UNINITIALIZED ");
			break;
		default:
			trace( "Unknown error " );
		}

	mFTResult->Reset();
};

void FTHelperContext::trace( const std::string &message ) 
{
	//ci::app::console() << message << "\n";
	OutputDebugStringA( ( message + "\n" ).c_str() );
}

//void FaceTracker::checkFaces(IFTImage* pColorImage, IFTImage* pDepthImage, int currentZoom, int viewOffset, int numberOfUsers) 
int FaceTracker::checkFaces(NUI_SKELETON_FRAME *skeleton, IFTImage *pColorImage, IFTImage *pDepthImage, int currentZoom, int viewOffset)
{

	if(!mContexts) {
		// for each user we want to make a new face tracker
		for( int i = 0; i < mNumUsers; i++) {
			//std::tr1::shared_ptr<FTHelperContext> context( new FTHelperContext() );
			//context->initialize( &mVideoCameraConfig, &mDepthCameraConfig );
			mContexts[i].initialize(&mVideoCameraConfig, &mDepthCameraConfig );
		}
	}

	HRESULT hr = S_OK;

	// we just loop through all of them and update whichever ones can be tracked or are being tracked
	for (UINT i=0; i<mNumUsers; i++)
    {

		if ( ( skeleton->SkeletonData + i )->eTrackingState != NUI_SKELETON_TRACKED && mContexts[i].mLastTrackSucceeded) {

			mContexts[i].mCountUntilFailure--;

			if(mContexts[i].mCountUntilFailure == 0) {
					mContexts[i].mLastTrackSucceeded = false;
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

        if (mContexts[i].mLastTrackSucceeded)
        {
            hr = mContexts[i].mFaceTracker->ContinueTracking(&sensorData, hint, mContexts[i].mFTResult);
        }
        else
        {
	        hr = mContexts[i].mFaceTracker->StartTracking(&sensorData, NULL, hint, mContexts[i].mFTResult);

			mContexts[i].mLastTrackSucceeded = SUCCEEDED(hr) && SUCCEEDED(mContexts[i].mFTResult->GetStatus());

			if(mContexts[i].mLastTrackSucceeded ) {
				mContexts[i].mSkeletonId = skeletonData->dwTrackingID;
				mContexts[i].mCountUntilFailure = 30;
				
			} else {
				mContexts[i].mFTResult->Reset();
			}
        }

        //mContexts[i].mLastTrackSucceeded = SUCCEEDED(hrFT) && SUCCEEDED(mContexts[i].mFTResult->GetStatus());

		if(mContexts[i].mLastTrackSucceeded == true) {
			// pass the handle to the face tracker
			std::cout << " getting a face " << std::endl;
			mContexts[i].mFaceTracker->GetFaceModel( &mContexts[i].mFTModel );
        }

        //SetCenterOfImage(mContexts[i].mFTResult);
    }

	return hr;
}

Vec2f FaceTracker::getCenterOfFace( UINT userId ) {

	Vec2f v;

	for( unsigned int i = 0; i < mNumUsers; i++) {
		if(mContexts[i].mSkeletonId == userId) {

			RECT faceCenter;
			mContexts[i].mFTResult->GetFaceRect(&faceCenter);
			v.set( (float) faceCenter.left + (faceCenter.right - faceCenter.left / 2), (float) faceCenter.top + (faceCenter.top - faceCenter.bottom / 2));
			return v;
		}
	}  

	return v;
}

uint32_t FaceTracker::getNumFaces()
{
	int numFaces = 0;
	for( unsigned int i = 0; i < mNumUsers; i++) {
		if(mContexts[i].mCountUntilFailure > 0)
			numFaces++;
	}
	return numFaces;
}

std::vector<uint32_t> FaceTracker::getFaceTriangles( UINT userId ) 
{
	std::vector<uint32_t> vec;
	for( unsigned int i = 0; i < mNumUsers; i++) {
		if(mContexts[i].mSkeletonId == userId) {

			FT_TRIANGLE *faceIndices;
			UINT pPointCount;

			mContexts[i].mFTModel->GetTriangles(&faceIndices, &pPointCount);

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
	for( unsigned int i = 0; i < mNumUsers; i++) {
		if(mContexts[i].mSkeletonId == userId) {
			for( unsigned int j = 0; j < mContexts[i].mFTModel->GetAUCount(); j++) {
				//units[] = 
			}
			
		}
	}
}

int FaceTracker::getProjectedShape(UINT userId, float &scale, Vec3f &rotation, Vec3f &translation, std::vector<Vec2f> &projectedFacePoints )
{
	// catch all errors
	HRESULT hr = S_OK;

	for( unsigned int i = 0; i < mNumUsers; i++) {
		if(mContexts[i].mSkeletonId == userId && mContexts[i].mLastTrackSucceeded) {

			UINT vertexCount = mContexts[i].mFTModel->GetVertexCount();
			FT_VECTOR2D* pPts2D = reinterpret_cast<FT_VECTOR2D*>(_malloca(sizeof(FT_VECTOR2D) * vertexCount));

			// first get all the data about the face
			FLOAT fscale, rotationXYZ[3], translationXYZ[3];
			mContexts[i].mFTResult->Get3DPose(&fscale, rotationXYZ, translationXYZ);

			// next animation units
			FLOAT *animationUnits;
			UINT auCount;
			hr = mContexts[i].mFTResult->GetAUCoefficients(&animationUnits, &auCount);

			if(hr != S_OK)
				return hr;
			
			// now look at the shape units
			FLOAT *shapeUnitCoeffiecients;
			UINT shapeUnitCount;
			BOOL converged;
			hr = mContexts[i].mFaceTracker->GetShapeUnits(&fscale, &shapeUnitCoeffiecients, &shapeUnitCount, &converged);

			if(hr != S_OK)
				return hr;

			// finally, actually get the projected 2d shape
			hr = mContexts[i].mFTModel->GetProjectedShape(&mVideoCameraConfig, mZoomFactor, POINT(), shapeUnitCoeffiecients, mContexts[i].mFTModel->GetSUCount(), animationUnits, auCount, 
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
	return mContexts[0].mLastTrackSucceeded;
}
		
bool FaceTracker::lastTrackSucceeded(int userId)
{
	for( int i = 0; i < mNumUsers; i++) {
		if(mContexts[i].mSkeletonId == userId) {
			return mContexts[i].mLastTrackSucceeded;
		}
	}
	return false;
}

void FaceTracker::startTracking( const FT_SENSOR_DATA *pSensorData, const RECT *pRoi, const FT_VECTOR3D headPoints[2])
{
	mContexts[0].mFaceTracker->StartTracking(pSensorData, pRoi, headPoints, mContexts[0].mFTResult);
}

void FaceTracker::continueTracking( const FT_SENSOR_DATA *pSensorData, const FT_VECTOR3D headPoints[2]) {
	mContexts[0].mFaceTracker->ContinueTracking(pSensorData, headPoints, mContexts[0].mFTResult);
}

void FaceTracker::startTracking( const FT_SENSOR_DATA *pSensorData, const RECT *pRoi, const FT_VECTOR3D headPoints[2], int skeletonId) {
	for( int i = 0; i < mNumUsers; i++) {
		if(mContexts[i].mSkeletonId == skeletonId)
			mContexts[0].mFaceTracker->StartTracking(pSensorData, pRoi, headPoints, mContexts[0].mFTResult);
	}
}
void FaceTracker::continueTracking( const FT_SENSOR_DATA *pSensorData, const FT_VECTOR3D headPoints[2], int skeletonId)
{
		for( int i = 0; i < mNumUsers; i++) {
		if(mContexts[i].mSkeletonId == skeletonId)
			mContexts[0].mFaceTracker->ContinueTracking(pSensorData, headPoints, mContexts[0].mFTResult);
	}
}

void FaceTracker::start(const DeviceOptions &deviceOptions)
{
	mDeviceOptions = deviceOptions;
	mFaceTrackingThread = CreateThread(NULL, 0, FaceTrackingStaticThread, (PVOID)this, 0, 0);
}

DWORD WINAPI FaceTracker::FaceTrackingStaticThread(PVOID lpParam)
{
    FaceTracker* context = static_cast<FaceTracker*>(lpParam);
    if (context)
    {
        return context->FaceTrackingThread();
    }
    return 0;
}

DWORD WINAPI FaceTracker::FaceTrackingThread()
{
	HRESULT hr;

    FT_CAMERA_CONFIG videoConfig;
    FT_CAMERA_CONFIG depthConfig;
    FT_CAMERA_CONFIG* pDepthConfig = NULL;

    // Try to get the Kinect camera to work
    //HRESULT hr = mSensor.Init(m_depthType, m_depthRes, m_bNearMode, FALSE, m_colorType, m_colorRes, m_bSeatedSkeleton);//
	mKinect = Kinect::create();
	
	// Stop, if capturing
	if ( mKinect->isCapturing() ) {
		mKinect->stop();
	}

	// Start Kinect
	mKinect->start( mDeviceOptions );

	if (mKinect->isCapturing())
    {
        mHasKinect = TRUE;
        //mKinect.GetVideoConfiguration(&videoConfig);
        //mKinect.GetDepthConfiguration(&depthConfig);
		
		depthConfig.Width = mKinect->getDeviceOptions().getDepthSize().x;
		depthConfig.Height = mKinect->getDeviceOptions().getDepthSize().y;
		depthConfig.FocalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS;

		videoConfig.Width = mKinect->getDeviceOptions().getVideoSize().x;
		videoConfig.Height = mKinect->getDeviceOptions().getVideoSize().y;
		videoConfig.FocalLength = NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS;

        pDepthConfig = &depthConfig;
    }
    else
    {
        
    }
    
    mContexts = new FTHelperContext[mNumUsers];
    if (mContexts != 0)
    {
        memset(mContexts, 0, sizeof(FTHelperContext) * mNumUsers);
    }
    else
    {
    }

    for (UINT i=0; i<mNumUsers;i++)
    {
        // Try to start the face tracker.
        mContexts[i].initialize( &videoConfig, &depthConfig );
        if (!mContexts[i].mFaceTracker)
        {
        }

        hr = mContexts[i].mFaceTracker->Initialize(&videoConfig, pDepthConfig, NULL, NULL); 
        if (FAILED(hr))
        {
        }
        mContexts[i].mFaceTracker->CreateFTResult(&mContexts[i].mFTResult);
        if (!mContexts[i].mFTResult)
        {
        }
        mContexts[i].mLastTrackSucceeded = false;
    }

    // Initialize the RGB image.
    mFTColorImage = FTCreateImage();

    if (!mFTColorImage || FAILED(hr = mFTColorImage->Allocate(videoConfig.Width, videoConfig.Height, FTIMAGEFORMAT_UINT8_B8G8R8X8)))
    {
        //return 6;
    }
    
    mFTDepthImage = FTCreateImage();
    if (!mFTDepthImage || FAILED(hr = mFTDepthImage->Allocate(depthConfig.Width, depthConfig.Height, FTIMAGEFORMAT_UINT16_D13P3)))
    {
        //return 7;
    }

    //SetCenterOfImage(NULL);

    while (true)
    {
        run();
        Sleep(16);
    }
    return 0;
}

uint32_t FaceTracker::addFaceTrackingCallback( const boost::function<void ( std::vector<Face>, const DeviceOptions& )> &callback )
{
	uint32_t id = mCallbacks.empty() ? 0 : mCallbacks.rbegin()->first + 1;
	mCallbacks.insert( std::make_pair( id, CallbackRef( new Callback( mSignalFaceTrack.connect( callback ) ) ) ) );
	return id;
}

// Get a video image and process it.
// We employ special code to associate a user ID with a tracker.

void FaceTracker::run()
{
    HRESULT hrFT = E_FAIL;

    if (mKinect->isCapturing() && mKinect->getFTColorImage())
    {
        HRESULT hrCopy = mKinect->getFTColorImage()->CopyTo(mFTColorImage, NULL, 0, 0);
        if (SUCCEEDED(hrCopy) && mKinect->getFTDepthImage())
        {
            hrCopy = mKinect->getFTDepthImage()->CopyTo(mFTDepthImage, NULL, 0, 0);
        }
        // Do face tracking
        if (SUCCEEDED(hrCopy))
        {
            FT_SENSOR_DATA sensorData(mFTColorImage, mFTDepthImage);

            for (UINT i=0; i<mNumUsers; i++)
            {


				std::vector<KinectSdk::Skeleton> skels = mKinect->getSkeletons();

				if (mContexts[i].mFTResult == 0 || skels.size() < i)
                {
                    mContexts[i].mLastTrackSucceeded = false;
                    continue;
                }

                FT_VECTOR3D hint[2];				
				/*hint[0].x = mKinect->getSkeletons().at(i).find(NUI_SKELETON_POSITION_SPINE)->second.getPosition().x;
				hint[0].y = mKinect->getSkeletons().at(i).find(NUI_SKELETON_POSITION_SPINE)->second.getPosition().y;
				hint[0].z = mKinect->getSkeletons().at(i).find(NUI_SKELETON_POSITION_SPINE)->second.getPosition().z;

				hint[1].x = mKinect->getSkeletons().at(i).find(NUI_SKELETON_POSITION_SPINE)->second.getPosition().x;
				hint[1].y = mKinect->getSkeletons().at(i).find(NUI_SKELETON_POSITION_SPINE)->second.getPosition().y;
				hint[1].z = mKinect->getSkeletons().at(i).find(NUI_SKELETON_POSITION_SPINE)->second.getPosition().z;*/


                if (mContexts[i].mLastTrackSucceeded)
                {
                    hrFT = mContexts[i].mFaceTracker->ContinueTracking(&sensorData, hint, mContexts[i].mFTResult);
                }
                else
                {
                    hrFT = mContexts[i].mFaceTracker->StartTracking(&sensorData, NULL, hint, mContexts[i].mFTResult);
                }
                mContexts[i].mLastTrackSucceeded = SUCCEEDED(hrFT) && SUCCEEDED(mContexts[i].mFTResult->GetStatus());
                if (mContexts[i].mLastTrackSucceeded)
                {
					mNewFaceTrackData = true;
                }
                else
                {
                    mContexts[i].mFTResult->Reset();
                }
            }
        }
    }
}

void FaceTracker::update() {

	mKinect->update();

	if ( mNewFaceTrackData ) {
		mSignalFaceTrack( mFaceData, mDeviceOptions );
		mNewFaceTrackData = false;
	}
}

void FaceTracker::removeCallback(uint32_t id)
{
	if(mCallbacks.count( id )) {
		mCallbacks.find( id )->second->disconnect();
		mCallbacks.erase( id );
	} else {
		mKinect->removeCallback(id);
	}
}

void FaceTracker::stop() {
	mKinect->stop();

	for ( int i = 0; i < mNumUsers; i++) {
		mContexts[i].mFTResult->Release();
		mContexts[i].mFTModel->Release();
		mContexts[i].mFaceTracker->Release();
	}

}

FaceTrackerRef FaceTracker::create() {
	return FaceTrackerRef( new FaceTracker( ) );
}

void FaceTracker::init(int rgbImageWidth, int rgbImageHeight, int depthWidth, int depthHeight, float zoom, int numUsers ) {

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

	/*context = new FTHelperContext();

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
				*/
	mNeedInitializeContexts = true;

}

void FaceTracker::GetClosestHint(FT_VECTOR3D* pHint3D)
{
    /*int selectedSkeleton = -1;
    float smallestDistance = 0;

    if (!pHint3D)
    {
        //return(E_POINTER);
		return;
    }

    if (pHint3D[1].x == 0 && pHint3D[1].y == 0 && pHint3D[1].z == 0)
    {
        // Get the skeleton closest to the camera
		for (int i = 0 ; i < mKinect->getSkeletons().size(); i++ )
        {
			//if (mKinect->getSkeletons().at(i).at(NUI_SKELETON_POSITION_HEAD) && 
			if(smallestDistance == 0 || mKinect->getSkeletons().at(i).find(NUI_SKELETON_POSITION_HEAD)->second.getPosition().z < smallestDistance)
            {
                smallestDistance = mKinect->getSkeletons().at(i).find(NUI_SKELETON_POSITION_HEAD)->second.getPosition().z;
                selectedSkeleton = i;
            }
        }
    }
    else
    {   // Get the skeleton closest to the previous position
        for (int i = 0 ; i < mKinect->getSkeletons().size() ; i++ )
        {
                float d = abs( mKinect->getSkeletons().at(i).find(NUI_SKELETON_POSITION_HEAD)->second.getPosition().x - pHint3D[1].x) +
                    abs( mKinect->getSkeletons().at(i).find(NUI_SKELETON_POSITION_HEAD)->second.getPosition().y - pHint3D[1].y) +
                    abs( mKinect->getSkeletons().at(i).find(NUI_SKELETON_POSITION_HEAD)->second.getPosition().z - pHint3D[1].z);
                if (smallestDistance == 0 || d < smallestDistance)
                {
                    smallestDistance = d;
                    selectedSkeleton = i;
                }
        }
    }
    if (selectedSkeleton == -1)
    {
        //return E_FAIL;
    }

	pHint3D[0].x = mKinect->getSkeletons().at(selectedSkeleton).find(NUI_SKELETON_POSITION_SPINE)->second.getPosition().x;
	pHint3D[0].y = mKinect->getSkeletons().at(selectedSkeleton).find(NUI_SKELETON_POSITION_SPINE)->second.getPosition().y;
	pHint3D[0].z = mKinect->getSkeletons().at(selectedSkeleton).find(NUI_SKELETON_POSITION_SPINE)->second.getPosition().z;

    pHint3D[1].x = mKinect->getSkeletons().at(selectedSkeleton).find(NUI_SKELETON_POSITION_HEAD)->second.getPosition().x;
	pHint3D[1].y = mKinect->getSkeletons().at(selectedSkeleton).find(NUI_SKELETON_POSITION_HEAD)->second.getPosition().y;
	pHint3D[1].z = mKinect->getSkeletons().at(selectedSkeleton).find(NUI_SKELETON_POSITION_HEAD)->second.getPosition().z;*/

    //return S_OK;
}
