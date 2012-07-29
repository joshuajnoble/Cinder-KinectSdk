
#pragma once

#include <FaceTrackLib.h>

#include "NuiApi.h"
#include "cinder\Cinder.h"
#include "cinder\Vector.h"
#include "cinder\Rect.h"
#include <vector>

// Kinect NUI wrapper for Cinder
//namespace KinectSdk
//{

	using namespace ci;

	class FTHelperContext
	{
	public:
		IFTFaceTracker*     m_pFaceTracker;
		IFTResult*          m_pFTResult;
		IFTModel*			m_pFTModel;
		FT_VECTOR3D         m_hint3D[2];
		bool                m_LastTrackSucceeded;
		int                 m_CountUntilFailure;
		UINT                m_SkeletonId;
	};

	class FaceTracker {

		public:

		FaceTracker( int rgbImageHeight, int rgbImageWidth, int depthWidth, int depthHeight, int numUsers );
		std::vector<FTHelperContext *> mContexts;
	
		// would be nice to get the middle of the face, no?
		Vec2f getCenterOfFace( UINT userId );
		std::vector<Vec2f> getFacePoints( UINT userId );
		int get3DPose(UINT userId, float *scale, Vec3f *rotation, Vec3f *translation );
		Rectf getFaceRect( UINT userId );
		void checkFaces(NUI_SKELETON_FRAME *skeleton, IFTImage *pColorImage, IFTImage *pDepthImage, int currentZoom, int viewOffset);

		bool lastTrackSucceeded();
		bool lastTrackSucceeded(int userId);

		void startTracking( const FT_SENSOR_DATA *pSensorData, const RECT *pRoi, const FT_VECTOR3D headPoints[2]);
		void continueTracking( const FT_SENSOR_DATA *pSensorData, const FT_VECTOR3D headPoints[2]);

		void startTracking( const FT_SENSOR_DATA *pSensorData, const RECT *pRoi, const FT_VECTOR3D headPoints[2], int skeletonId);

		void continueTracking( const FT_SENSOR_DATA *pSensorData, const FT_VECTOR3D headPoints[2], int skeletonId);

	};
//};