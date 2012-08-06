
#pragma once

#include <FaceTrackLib.h>
#include "Kinect.h"

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
	using namespace KinectSdk;

	typedef NUI_SKELETON_BONE_ROTATION		BoneRotation;
	typedef NUI_IMAGE_RESOLUTION			ImageResolution;
	typedef NUI_SKELETON_POSITION_INDEX		JointName;

	class Face
	{
	
		public:

		std::vector<ci::Vec2f> screenPositions;
		ci::Vec3f rotation;
		ci::Vec3f transform;
		float scale;

		ci::Rectf faceRect;

		void trace( const std::string &message );
		std::map<std::string, float> animationUnitData;

	};

	class FTHelperContext
	{
	public:

		FTHelperContext() {}; // does nothing
		void initialize( FT_CAMERA_CONFIG *videoCameraConfig, FT_CAMERA_CONFIG *depthCameraConfig);
		void trace( const std::string &message );
		IFTFaceTracker		*mFaceTracker;
		IFTResult           *mFTResult;
		IFTModel			*mFTModel;
		FT_VECTOR3D         mHint3D[2];
		bool                mLastTrackSucceeded;
		int                 mCountUntilFailure;
		UINT                mSkeletonId;
	};

	class FaceTracker;
	typedef std::shared_ptr<FaceTracker>			FaceTrackerRef;

	class FaceTracker {

		public:

		// constructor
		FaceTracker() {}
			
		void init( int rgbImageWidth = 640, int rgbImageHeight = 480, int depthWidth = 320, int depthHeight = 240, float zoom = 1.0, int numUsers = NUI_SKELETON_COUNT );

		//! Creates pointer to instance of Kinect
		static FaceTrackerRef				create();	

		//! get the vert indices for the triangles of a face
		std::vector<uint32_t> getFaceTriangles( UINT userId );
	
		//! return the number of faces that we'r tracking
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

		//! Start the Kinect and the Face Tracking
		void start( const DeviceOptions &deviceOptions = DeviceOptions() );
		//! update both the Kinect and the FaceTracking
		void update();
		
		//! Adds depth image callback to the Kinect
		uint32_t						addDepthCallback( const boost::function<void ( ci::Surface16u, const DeviceOptions& )> &callback );
		//! Adds skeleton tracking callback to the Kinect
		uint32_t						addSkeletonTrackingCallback( const boost::function<void ( std::vector<Skeleton>, const DeviceOptions& )> &callback );
		//! Adds video image callback to the Kinect
		uint32_t						addVideoCallback( const boost::function<void ( ci::Surface8u, const DeviceOptions& )> &callback );
		//! Adds a callback for face tracking to this
		uint32_t						addFaceTrackingCallback( const boost::function<void ( std::vector<Face>, const DeviceOptions& )> &callback );

		//! Adds depth image callback to the Kinect
		template<typename T> 
		inline uint32_t					addDepthCallback( void ( T::*callbackFunction )( ci::Surface16u surface, const DeviceOptions& deviceOptions ), T *callbackObject )
		{
			//return addDepthCallback( boost::function<void ( ci::Surface16u, const DeviceOptions& )>( boost::bind( callbackFunction, callbackObject, ::_1, ::_2 ) ) );
			return mKinect->addDepthCallback<T>(callbackFunction, callbackObject);
		}
		//! Adds skeleton tracking callback to the Kinect
		template<typename T> 
		inline uint32_t					addSkeletonTrackingCallback( void ( T::*callbackFunction )( std::vector<Skeleton> skeletons, const DeviceOptions &deviceOptions ), T *callbackObject )
		{
			//return addSkeletonTrackingCallback( boost::function<void ( std::vector<Skeleton>, const DeviceOptions& )>( boost::bind( callbackFunction, callbackObject, ::_1, ::_2 ) ) );
			return mKinect->addSkeletonTrackingCallback<T>(callbackFunction, callbackObject);
		}
		//! Adds video image callback to the Kinect
		template<typename T> 
		inline uint32_t					addVideoCallback( void ( T::*callbackFunction )( ci::Surface8u surface, const DeviceOptions& deviceOptions ), T *callbackObject ) 
		{
			//return addVideoCallback( boost::function<void ( ci::Surface8u, const DeviceOptions& )>( boost::bind( callbackFunction, callbackObject, ::_1, ::_2 ) ) );
			return mKinect->addVideoCallback<T>(callbackFunction, callbackObject);
		}

		//! Adds face tracking callback to the Kinect
		template<typename T> 
		inline uint32_t					addFaceTrackingCallback( void ( T::*callbackFunction )( std::vector<Face> faces, const DeviceOptions& deviceOptions ), T *callbackObject ) 
		{
			return addFaceTrackingCallback( boost::function<void ( std::vector<Face>, const DeviceOptions& )>( boost::bind( callbackFunction, callbackObject, ::_1, ::_2 ) ) );
		}

		//! Adds face tracking callback.
		template<typename T> 
		inline uint32_t					addUserSelectionCallback( void ( T::*callbackFunction )( bool sucess, const DeviceOptions& deviceOptions ), T *callbackObject ) 
		{
			return addFaceTrackingCallback( boost::function<void ( bool sucess, const DeviceOptions& )>( boost::bind( callbackFunction, callbackObject, ::_1, ::_2 ) ) );
		}

		void setDeviceOptions(KinectSdk::DeviceOptions &options);

				//! Removes callback.
		void							removeCallback( uint32_t id );

		KinectRef						getKinect() { return mKinect; } // so you can just access it easily

		void stop();
		void run();

	private:

		KinectSdk::CallbackList			mCallbacks;

		KinectSdk::DeviceOptions		mDeviceOptions;

		FTHelperContext					*mContexts;

		IFTImage		                *mFTColorImage;
		IFTImage						*mFTDepthImage;
		std::vector<Face>				mFaceData;
		KinectSdk::KinectRef			mKinect;

		bool mHasKinect;
		bool mNewFaceTrackData;

		bool lastTrackSucceeded();
		bool lastTrackSucceeded(int userId);

		void startTracking( const FT_SENSOR_DATA *pSensorData, const RECT *pRoi, const FT_VECTOR3D headPoints[2]);
		void continueTracking( const FT_SENSOR_DATA *pSensorData, const FT_VECTOR3D headPoints[2]);

		void startTracking( const FT_SENSOR_DATA *pSensorData, const RECT *pRoi, const FT_VECTOR3D headPoints[2], int skeletonId);
		void continueTracking( const FT_SENSOR_DATA *pSensorData, const FT_VECTOR3D headPoints[2], int skeletonId);

		boost::signals2::signal<void ( std::vector<Face>, const DeviceOptions& )>			mSignalFaceTrack;

		FT_CAMERA_CONFIG mVideoCameraConfig, mDepthCameraConfig;

		DWORD WINAPI FaceTrackingThread();
		static DWORD WINAPI FaceTrackingStaticThread(PVOID lpParam);
		//static void SelectUserToTrack(KinectSensor * pKinectSensor, UINT nbUsers, FTHelperContext* pUserContexts);

		float mZoomFactor, mViewOffset;
		int inited;
		int mNumUsers;
		bool mNeedInitializeContexts;

		HANDLE mFaceTrackingThread;

		
		void GetClosestHint(FT_VECTOR3D* pHint3D);

	};
//};