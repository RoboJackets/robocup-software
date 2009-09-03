#pragma once

#include <string>
#include <vector>
#include <list>
#include <map>

#include <Geometry2d/TransformMatrix.hpp>
#include <Geometry2d/Point.hpp>
#include <Utils.hpp>

#include "Role.hpp"
#include "Play.hpp"
#include "Robot.hpp"
#include "Named_Object.hpp"
#include "GameplayModule.hpp"

namespace Gameplay
{
    class Parameter;

    class Behavior
    {
    public:
        Behavior(GameplayModule *gameplay, Role *role = 0);
        virtual ~Behavior();

        GameplayModule *gameplay() const
        {
            return _gameplay;
        }
        
        const Packet::LogFrame::Ball &ball() const
        {
            return gameplay()->state()->ball;
        }
        
        const std::string &name() const { return _name; }
        Role *role() const { return _role; }
        Robot *robot() const { return _robot; }
        ObstacleGroup *obstacles() const  { return &gameplay()->state()->self[_robot->id()].obstacles; }
        bool assigned() const { return _robot != 0; }

        Parameter *parameter(const std::string &name) const
        {
            return Utils::map_lookup(_parameters, name);
        }

        // Returns how important it is for this tactic to select the robot.
        // The tactic in a role with the highest priority() will get to select the
        // robot for that role.
        //
        // If two or more tactics have equal priority, the first one declared in the role
        // will choose the robot.
        //
        // The default implementation always returns zero.
        virtual float priority();

        // Returns a score judging how good the given robot is for this role containing this tactic.
        // This is called before robots are assigned to roles (and thus before start()).
        // The robot which receives the lowest score will be assigned to this tactic's role.
        //
        // The default implementation always returns 0.
        virtual float score(Robot *robot);

        // Returns the best robot for this behavior (the one with the lowest score).
        // This is guaranteed to return a robot if any are available.
        Robot *selectRobot();

        // Assigns a robot to this tactic.
        // stop() and start() are called as appropriate.
        void robot(Robot *robot);

        // Called when this becomes the current tactic for a role.
        // The default implementation does nothing.
        virtual void start();

        // Called when this stops being the current tactic for a role
        // (another tactic will become current or the play has ended).
        // The default implementation does nothing.
        virtual void stop();

        // Called each frame when this tactic is current.
        // The default implementation does nothing.
        virtual void run();

        // Returns true if this tactic allows the play to advance.
        // The default implementation always returns true.
        virtual bool done();

    protected:
        friend class Parameter;
        friend class BehaviorFactory;

        GameplayModule *_gameplay;
        
        // This is set by the Factory when the Tactic is created,
        // so the name only has to be given when the Factory is created.
        std::string _name;

        // The role that contains this behavior.
        // This is null for the goalie or for a behavior created by another behavior.
        Role *_role;

        // The robot assigned to this tactic or null if none.
        // If this tactic is running, it is guaranteed to have a robot.
        Robot *_robot;

        typedef std::map<std::string, Parameter *> Parameter_Map;
        Parameter_Map _parameters;
    };

    // This abstract class is the base for all tactic factories.
    // A tactic factory creates a tactic of a particular type.
    // Tactic factories are stored in a mapping from name to factory,
    // which is used by the parser to create tactics by name.
    class BehaviorFactory: public Named_Object<BehaviorFactory>
    {
    public:
        BehaviorFactory(const char *name): Named_Object<BehaviorFactory>(name) {}
        virtual ~BehaviorFactory();

        // Creates a new tactic of the appropriate type.
        virtual Behavior *create(GameplayModule *gameplay, Role *role) = 0;

        static Behavior *create(const std::string &name, GameplayModule *gameplay, Role *role);

    protected:
        // This allows Factory to be a friend of Behavior so Behavior::_name can be protected.
        void setName(Behavior *behavior)
        {
            behavior->_name = _name;
        }
    };

    // This class creates tactics of a particular type.
    // Declare a global variable like this:
    //      Tactics::Factory_Type<Tactics::Move> move_tactic("move");
    // and the factory will be registered by its constructor.
    // The parser will find the factory and create tactics of this type
    // when plays are loaded.
    template<class Behavior_Class>
    class BehaviorFactoryType: public BehaviorFactory
    {
    public:
        BehaviorFactoryType(const char *name): BehaviorFactory(name) {}

        virtual Behavior *create(GameplayModule *gameplay, Role *role)
        {
            Behavior *behavior = new Behavior_Class(gameplay, role);
            setName(behavior);
            return behavior;
        }
    };
}
