#pragma once

#include <ctime>
#include <Eigen/Core>

namespace HUREL {
	namespace Compton {
		enum eInterationType
		{
			NONE,
			COMPTON,
			CODED			
		};

		/// <summary>
		/// meter and MeV
		/// </summary>
		struct InteractionData
		{
			Eigen::Vector3d RelativeInteractionPoint;
			Eigen::Vector3d TransformedInteractionPoint; 	
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



