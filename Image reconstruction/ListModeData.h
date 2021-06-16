#pragma once

#include <ctime>
#include <Eigen/Core>

namespace HUREL {
	namespace Compton {
		enum class eInterationType
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
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			Eigen::Vector4d RelativeInteractionPoint;
			Eigen::Vector4d TransformedInteractionPoint; 	
			double InteractionEnergy;  
		};

		struct ListModeData
		{
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			eInterationType Type;
			InteractionData Scatter;
			InteractionData Absorber;
			time_t InterationTime;
			Eigen::Matrix4d DetectorTransformation;
		};

	}
}



