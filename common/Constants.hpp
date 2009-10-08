// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

/// all distances in meters
/// all times in seconds
/// all weights in killograms

namespace Constants
{
    const int Robots_Per_Team = 5;
	
	// Number of robots controlled by a play (all except the goalie)
	const int Play_Robots = Robots_Per_Team - 1;
    
    namespace Ball
    {
        const float Diameter = 0.043f;
        const float Radius = Diameter/2.0f;
        const float Mass = 0.048f;
    }

    namespace Field
    {
        const float Length = 6.05f;
        const float Width = 4.05f;
        const float Border = 0.25f;

        const float LineWidth = 0.01f;

        const float GoalWidth = 0.7f;
        const float GoalDepth = 0.2f;
        const float GoalHeight = 0.2f;

        /** Disstance of the penalty marker from the goal line */
        const float PenaltyDist = 0.450f;
        const float PenaltyDiam = 0.010f;

        /** Radius of the goal arcs */
        const float ArcRadius = 0.5f;
        
        /** diameter of the center circle */
        const float CenterRadius = 0.5f;
        const float CenterDiameter = CenterRadius * 2.0f;
        
        /** flat area for defence markings */
        const float GoalFlat = 0.35f;
    }

    namespace Floor
    {
        const float Length = Field::Length + 2.0 * Field::Border;
        const float Width = Field::Width + 2.0 * Field::Border;

        const float Aspect = Width/Length;
    }

    namespace Robot
    {
        const float Diameter = 0.180f;
        const float Radius = Diameter/2.0f;
        const float Height = 0.150f;
    }
}
