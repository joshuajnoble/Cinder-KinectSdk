
#pragma once

#include <FaceTrackLib.h>

#include "NuiApi.h"
#include "cinder\Cinder.h"
#include "cinder\Vector.h"
#include "cinder\Rect.h"
#include <vector>
#include <map>
#include <sstream>

// Kinect NUI wrapper for Cinder
//namespace KinectSdk
//{

	using namespace ci;

	/*class FTHelperContext
	{
	public:

		void initialize( FT_CAMERA_CONFIG *videoCameraConfig, FT_CAMERA_CONFIG *depthCameraConfig) {

			mFaceTracker = FTCreateFaceTracker();

			if(!mFaceTracker) {
				trace( " Can't create face tracker " );
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

		void trace( const std::string &message ) 
		{
			//ci::app::console() << message << "\n";
			OutputDebugStringA( ( message + "\n" ).c_str() );
		}

		IFTFaceTracker		*mFaceTracker;
		IFTResult           *mFTResult;
		IFTModel			*mFTModel;
		FT_VECTOR3D         mHint3D[2];
		bool                mLastTrackSucceeded;
		int                 mCountUntilFailure;
		UINT                mSkeletonId;
	};*/

	struct FTHelperContext {
		IFTFaceTracker		*mFaceTracker;
		IFTResult           *mFTResult;
		IFTModel			*mFTModel;
		FT_VECTOR3D         mHint3D[2];
		bool                mLastTrackSucceeded;
		int                 mCountUntilFailure;
		UINT                mSkeletonId;
	};

	class FaceTracker {

		public:

		FaceTracker( int rgbImageWidth = 640, int rgbImageHeight = 480, int depthWidth = 320, int depthHeight = 240, float zoom = 1.0, int numUsers = NUI_SKELETON_COUNT );
		
		//std::tr1::shared_ptr<std::vector< std::tr1::shared_ptr<FTHelperContext>>> mContexts;
		std::vector< FTHelperContext* > *mContexts;

		FTHelperContext* context;

		std::vector<uint32_t> getFaceTriangles( UINT userId );
	
		uint32_t getNumFaces();

		// would be nice to get the middle of the face, no?
		Vec2f getCenterOfFace( UINT userId );
		//! Just get the projected face points
		int getProjectedShape(UINT userId, float &scale, Vec3f &rotation, Vec3f &translation, std::vector<Vec2f> &projectedFacePoints );
		//! Just get the rect that someone where the face has been found
		Rectf getFaceRect( UINT userId );
		//! update all the facetracker instances
		int checkFaces(NUI_SKELETON_FRAME *skeleton, IFTImage *pColorImage, IFTImage *pDepthImage, int currentZoom, int viewOffset);
		//! just the animation units, useful if you want to use this as a NUI of some sort
		void getAnimationUnits( std::map<std::string, float> &units, UINT userId);

		bool lastTrackSucceeded();
		bool lastTrackSucceeded(int userId);

		void startTracking( const FT_SENSOR_DATA *pSensorData, const RECT *pRoi, const FT_VECTOR3D headPoints[2]);
		void continueTracking( const FT_SENSOR_DATA *pSensorData, const FT_VECTOR3D headPoints[2]);

		void startTracking( const FT_SENSOR_DATA *pSensorData, const RECT *pRoi, const FT_VECTOR3D headPoints[2], int skeletonId);
		void continueTracking( const FT_SENSOR_DATA *pSensorData, const FT_VECTOR3D headPoints[2], int skeletonId);

	private:

		FT_CAMERA_CONFIG mVideoCameraConfig, mDepthCameraConfig;

		float mZoomFactor, mViewOffset;
		int inited;
		int mNumUsers;
		bool mNeedInitializeContexts;

	};
//};