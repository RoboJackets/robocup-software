#pragma once

typedef enum
{
    UnknownTeam = -1,
	Yellow = 0,
 	Blue = 1
} Team;

static inline Team opponentTeam(Team t)
{
	if (t == Blue)
	{
		return Yellow;
	}
	else if (t == Yellow)
	{
		return Blue;
	}
	
	return UnknownTeam;
}

static inline const char* teamName(Team t)
{
	if (t == Blue)
	{
		return "Blue";
	}
	else if (t == Yellow)
	{
		return "Yellow";
	}
	
	return "UnkownTeam";
}
 
