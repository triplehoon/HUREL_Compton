#pragma once

#include <open3d/geometry/PointCloud.h>
#include <ctime>

namespace HUREL {
	namespace Compton {
		enum eInterationType
		{
			NONE,
			COMPTON,
			CODED			
		};

		struct InteractionData
		{
			double RelativeInteractionPointX;
			double RelativeInteractionPointY;
			double RelativeInteractionPointZ;
			double TransformedInteractionPointX;
			double TransformedInteractionPointY;
			double TransformedInteractionPointZ;			
			double InteractionEnergy;  
		};

		struct ListModeData
		{
			eInterationType Type;
			InteractionData Scatter;
			InteractionData Absorber;
			time_t InterationTime;
			Eigen::Matrix4d DetectorTransformation;
		};

	}
}



