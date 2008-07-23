#include "Play.hpp"
#include "Play_Lexer.hpp"
#include "Play_Parser.hpp"
#include "Role.hpp"
#include "Robot.hpp"
#include "Predicate.hpp"
#include "tactics/Tactics.hpp"

#include <fstream>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace boost;

float Play::Predicate_Preference::value() const
{
    if (predicate && predicate->evaluate())
    {
        return weight;
    } else {
        return 0;
    }
}

float Play::Completed_Preference::value() const
{
    int n = _play->num_started();
    if (n)
    {
        return (float)_play->num_completed() / (float)n;
    } else {
        return 0;
    }
}

////////

Play::Play()
{
    init();
}

Play::Play(const char *filename)
{
    init();
    
    ifstream file(filename);
    Play_Lexer lexer(file);
    Play_Parser parser(lexer);
    
    parser.play_file(this);
    
    default_preferences();
}

void Play::init()
{
    _status = Terminated;
    _stats_done = true;
    _preference_value = 0;
    timeout = -1;
    
    reset_stats();
    
    restart = true;
}

Play::~Play()
{
    BOOST_FOREACH(Role *role, _roles)
    {
        delete role;
    }
    
    BOOST_FOREACH(Preference *pref, preferences)
    {
        delete pref;
    }
    
    BOOST_FOREACH(Opponent *opp, _opponents)
    {
        delete opp;
    }
}

void Play::default_preferences()
{
    if (preferences.empty())
    {
        preferences.push_back(new Completed_Preference(this));
    }
}

void Play::start()
{
    BOOST_FOREACH(Role *role, _roles)
    {
        Robot *robot = role->select_robot();
        if (robot)
        {
            // Start the role (and its first tactic)
            role->robot(robot);
        }
    }
    
    // Find opponents
    BOOST_FOREACH(Opponent *opp, _opponents)
    {
        opp->select();
    }
    
    _status = Running;
    
    // Set robot parameters
    BOOST_FOREACH(const Param_Name_Ref &ref, name_params)
    {
        Robot *robot = Robot::find(ref.name);
        if (!robot)
        {
            printf("No robot \"%s\"\n", ref.name.c_str());
            _status = Aborted;
        }
        ref.param->set(robot);
    }
    
    _start_time.start();
    _stats_done = false;
    _num_started++;
}

void Play::run()
{
    // Run all roles
    bool all_done = true;
    BOOST_FOREACH(Role *role, _roles)
    {
        if (role->assigned())
        {
            // If we lost a robot, abort the play.
            if (!role->robot()->visible())
            {
                if (!_stats_done)
                {
                    printf("Aborted: lost robot %d\n", role->robot()->id());
                }
                
                _status = Aborted;
                all_done = false;
                break;
            } else {
                // Run the current tactic for this role.
                Tactics::Base *tactic = role->current_tactic();
                if (tactic)
                {
                    tactic->run();
                    all_done &= tactic->done();
                }
            }
        }
    }
    
    if (all_done)
    {
        // All tactics are done, so advance all roles.
        all_done = true;
        BOOST_FOREACH(Role *role, _roles)
        {
            if (role->assigned())
            {
                role->advance();
                all_done &= role->done();
            }
        }
    }
    
    // all_done now indicates if all *roles* have finished.
    
    // Check if the play has finished
    if (terminate.any())
    {
        _status = Terminated;
    } else if (all_done)
    {
        _status = Completed;
    } else if (timeout >= 0 && (float)_start_time.elapsed() >= (timeout * 1000.0))
    {
        _status = Timeout;
    }
    
    // Update statistics
    if (_status != Running && !_stats_done)
    {
        _stats_done = true;
        switch (_status)
        {
            case Timeout:
                _num_timeouts++;
                break;
            
            case Terminated:
                _num_terminated++;
                break;
            
            case Completed:
                _num_completed++;
                break;
            
            case Aborted:
                _num_aborted++;
                break;
            
            default:
                break;
        }
    }
}

void Play::stop()
{
    _status = Terminated;
    _stats_done = true;
    
    BOOST_FOREACH(Role *role, _roles)
    {
        if (role->assigned())
        {
            // Unassign and stop this role
            role->robot(0);
        }
    }
    
    // Unassign opponents
    for (int i = 0; i < 5; ++i)
    {
        Robot::opp[i].opponent(0);
    }
}

Role *Play::role(const std::string &name)
{
    BOOST_FOREACH(Role *role, _roles)
    {
        if (role->name() == name)
        {
            return role;
        }
    }
    
    return 0;
}

void Play::reset_stats()
{
    _num_started = 0;
    _num_timeouts = 0;
    _num_terminated = 0;
    _num_completed = 0;
}

void Play::update_preference()
{
    _preference_value = 0;
    BOOST_FOREACH(const Preference *term, preferences)
    {
        _preference_value += term->value();
    }
}

bool Play::add_preference(const std::string &name, float weight)
{
    if (name == "completed")
    {
        preferences.push_back(new Completed_Preference(this));
    } else {
        // Check for predicates
        Predicate *pred = Predicate::find(name);
        if (pred)
        {
            preferences.push_back(new Predicate_Preference(pred, weight));
            return true;
        }
    }
    
    return false;
}

void Play::opponent(const std::string &id, const std::string &desc)
{
    Opponent *opp = 0;
    
    if (desc == "near_ball")
    {
        opp = new Opponent_Near_Ball(id);
    }
    else if (desc == "near_goal")
    {
        opp = new Opponent_Near_Goal(id);
    }
    else 
    {
        throw runtime_error(str(format("Bad opponent type \"%s\"") % desc));
    }
    
    if (opp)
    {
        _opponents.push_back(opp);
    }
}
