
#include <FaceTrackLib.h>

#include <boost\thread\thread.hpp>
#include "Kinect.h"
#include "cinder\Cinder.h"
#include "cinder\Vector.h"
#include "cinder\Rect.h"

struct FTHelperContext
{
    IFTFaceTracker*     m_pFaceTracker;
    IFTResult*          m_pFTResult;
	IFTModel*			m_pFTModel;
    FT_VECTOR3D         m_hint3D[2];
    bool                m_LastTrackSucceeded;
    int                 m_CountUntilFailure;
    UINT                m_SkeletonId;
};

using namespace KinectSdk;
using namespace ci;

class FaceTracker {

public:

	FaceTracker(Kinect *kinect);
	std::vector<FTHelperContext *> mContexts;
	
	// would be nice to get the middle of the face, no?
	Vec2f getCenterOfFace( UINT userId );

	std::vector<Vec2f> getFacePoints( UINT userId );
	int get3DPose(UINT userId, float *scale, Vec3f *rotation, Vec3f *translation );
	Rectf getFaceRect( UINT userId );

	void checkFaces(NUI_SKELETON_FRAME *skeleton, IFTImage *pColorImage, IFTImage *pDepthImage, int currentZoom, int viewOffset);


private:

	std::thread	mThread;

};
