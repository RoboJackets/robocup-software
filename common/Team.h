#ifndef TEAM_H_
#define TEAM_H_

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
	
	return Blue;
}

static inline const char* teamToA(Team t)
{
	if (t == Blue)
	{
		return "Blue";
	}
	
	return "Yellow";
}

#endif /*TEAM_H_*/
 
