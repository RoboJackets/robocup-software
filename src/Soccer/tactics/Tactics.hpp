#ifndef _TACTICS__TACTICS_H_
#define _TACTICS__TACTICS_H_

#include <string>
#include <vector>
#include <list>
#include <map>

#include <Geometry/TransformMatrix.hpp>
#include <Geometry/Point2d.hpp>
#include <Packet/MotionCmd.hpp>
#include <Packet/VisionData.hpp>

#include "../map_utils.hpp"
#include "../Robot.hpp"
#include "../Named_Object.hpp"

class Role;
class Robot;

namespace Tactics
{
    class Parameter;

    class Base
    {
    public:
        Base(Role *role);
        virtual ~Base();

        const std::string &name() const { return _name; }
        Role *role() const { return _role; }
        Robot *robot() const { return _robot; }
        bool assigned() const { return _robot != 0; }

        Parameter *parameter(const std::string &name) const
        {
            return map_lookup(_parameters, name);
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

        // Returns the best robot for this tactic (this one with the lowest score).
        // This is guaranteed to return a robot if any are available.
        Robot *select_robot();

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
        friend class Factory;

        // This is set by the Factory when the Tactic is created,
        // so the name only has to be given when the Factory is created.
        std::string _name;

        // The role that contains this tactic.
        // This is null for the goalie.
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
    class Factory: public Named_Object<Factory>
    {
    public:
        Factory(const char *name): Named_Object<Factory>(name) {}
        virtual ~Factory();

        // Creates a new tactic of the appropriate type.
        virtual Base *create(Role *role) = 0;

        static Base *create(const std::string &name, Role *role);

    protected:
        // This allows Factory to be a friend of Base so Base::_name can be protected.
        void set_name(Base *tactic)
        {
            tactic->_name = _name;
        }
    };

    // This class actually creates tactics of a particular type.
    // Declare a global variable like this:
    //      Tactics::Factory_Type<Tactics::Move> move_tactic("move");
    // and the factory will be registered by its constructor.
    // The parser will find the factory and create tactics of this type
    // when plays are loaded.
    template<class Tactic_Class>
    class Factory_Type: public Factory
    {
    public:
        Factory_Type(const char *name): Factory(name) {}

        virtual Base *create(Role *role)
        {
            Base *tactic = new Tactic_Class(role);
            set_name(tactic);
            return tactic;
        }
    };
}

#endif // _TACTICS__TACTICS_H_
