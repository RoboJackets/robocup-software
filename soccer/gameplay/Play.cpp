#include "Play.hpp"
#include "Play_Lexer.hpp"
#include "Play_Parser.hpp"
#include "Role.hpp"
#include "Robot.hpp"
#include "Predicate.hpp"
#include "Behavior.hpp"
#include "GameplayModule.hpp"

#include <fstream>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace boost;

void Gameplay::Play_Parser::reportError(const antlr::RecognitionException &e)
{
    // By default, this is just printed.  Need to re-throw the exception
    // to make sure parsing fails.
    throw e;
}

float Gameplay::Play::Predicate_Preference::value() const
{
    if (predicate && predicate->evaluate())
    {
        return weight;
    } else {
        return 0;
    }
}

float Gameplay::Play::Completed_Preference::value() const
{
    int n = _play->numStarted();
    if (n)
    {
        return (float)_play->numCompleted() / (float)n;
    } else {
        return 0;
    }
}

////////

Gameplay::Play::Play(GameplayModule *gameplay)
{
    init(gameplay);
}

Gameplay::Play::Play(GameplayModule *gameplay, const char *filename)
{
    init(gameplay);
    
    ifstream file(filename);
    Play_Lexer lexer(file);
    Play_Parser parser(lexer);
    
    parser.play_file(this);
    
    defaultPreferences();
}

void Gameplay::Play::init(GameplayModule *gameplay)
{
    _gameplay = gameplay;
    _status = Terminated;
    _statsDone = true;
    _numStarted = 0;
    _numTimeouts = 0;
    _numTerminated = 0;
    _numCompleted = 0;
    _numAborted = 0;
    _preferenceValue = 0;
    timeout = -1;
    
    resetStats();
    
    restart = true;
}

Gameplay::Play::~Play()
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

void Gameplay::Play::defaultPreferences()
{
    if (preferences.empty())
    {
        preferences.push_back(new Completed_Preference(this));
    }
}

void Gameplay::Play::start()
{
    BOOST_FOREACH(Role *role, _roles)
    {
        Robot *robot = role->selectRobot();
        if (robot)
        {
            // Start the role (and its first behavior)
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
    BOOST_FOREACH(const Param_Name_Ref &ref, nameParams)
    {
        Robot *robot = _gameplay->find(ref.name);
        if (!robot)
        {
            printf("No robot \"%s\"\n", ref.name.c_str());
            _status = Aborted;
        }
        ref.param->set(robot);
    }
    
    _startTime.start();
    _statsDone = false;
    _numStarted++;
}

void Gameplay::Play::run()
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
                if (!_statsDone)
                {
                    printf("Aborted: lost robot %d\n", role->robot()->id());
                }
                
                _status = Aborted;
                all_done = false;
                break;
            } else {
                // Run the current behavior for this role.
                Behavior *behavior = role->currentBehavior();
                if (behavior)
                {
                    behavior->run();
                    all_done &= behavior->done();
                }
            }
        }
    }
    
    if (all_done)
    {
        // All behaviors are done, so advance all roles.
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
    if (all_done)
    {
        _status = Completed;
    } else if (terminate.any())
    {
        _status = Terminated;
    } else if (!require.all())
    {
        // Check for this after normal terminations in case both apply (e.g. for restarts).
        _status = Aborted;
    } else if (timeout >= 0 && (float)_startTime.elapsed() >= (timeout * 1000.0))
    {
        _status = Timeout;
    }
    
    // Update statistics
    if (_status != Running && !_statsDone)
    {
        _statsDone = true;
        switch (_status)
        {
            case Timeout:
                _numTimeouts++;
                break;
            
            case Terminated:
                _numTerminated++;
                break;
            
            case Completed:
                _numCompleted++;
                break;
            
            case Aborted:
                _numAborted++;
                break;
            
            default:
                break;
        }
    }
}

void Gameplay::Play::stop()
{
    _status = Terminated;
    _statsDone = true;
    
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
        _gameplay->opp[i]->opponent(0);
    }
}

Gameplay::Role *Gameplay::Play::role(const std::string &name)
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

void Gameplay::Play::resetStats()
{
    _numStarted = 0;
    _numTimeouts = 0;
    _numTerminated = 0;
    _numCompleted = 0;
}

void Gameplay::Play::updatePreference()
{
    _preferenceValue = 0;
    BOOST_FOREACH(const Preference *term, preferences)
    {
        _preferenceValue += term->value();
    }
}

bool Gameplay::Play::addPreference(const std::string &name, float weight)
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

void Gameplay::Play::opponent(const std::string &id, const std::string &desc)
{
    Opponent *opp = 0;
    
    if (desc == "near_ball")
    {
        opp = new Opponent_Near_Ball(_gameplay, id);
    }
    else if (desc == "near_goal")
    {
        opp = new Opponent_Near_Goal(_gameplay, id);
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
