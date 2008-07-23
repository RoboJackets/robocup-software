#ifndef _OPPONENT_HPP_
#define _OPPONENT_HPP_

#include <string>

class Robot;

class Opponent
{
public:
    Opponent(const std::string &name);
    virtual ~Opponent();
    
    void select();
    
    const std::string &name() const { return _name; }
    Robot *robot() const { return _robot; }

protected:
    // Lower scores are better.
    virtual float score(Robot *robot) const = 0;
    
    std::string _name;
    Robot *_robot;
};

class Opponent_Near_Ball: public Opponent
{
	public:
		Opponent_Near_Ball(const std::string &name): Opponent(name) {}
	
	protected:
		virtual float score(Robot *robot) const;
};

class Opponent_Near_Goal: public Opponent
{
	public:
		Opponent_Near_Goal(const std::string &name): Opponent(name) {}
	
	protected:
		virtual float score(Robot *robot) const;
};

#endif // _OPPONENT_HPP_
