#include "Playbook.hpp"
#include "Play.hpp"
#include "Named_Matrix.hpp"
#include "Behavior.hpp"
#include "GameplayModule.hpp"

#include <antlr/ANTLRException.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace antlr;
using namespace boost;
using namespace boost::filesystem;

Gameplay::Playbook::Playbook(GameplayModule *gameplay)
{
    _gameplay = gameplay;
    _goalie = 0;
    _currentPlay = 0;
    _needStart = false;
    _logged_end = false;
}

void Gameplay::Playbook::loadDir(const string &path)
{
    directory_iterator end;
    for (directory_iterator i(path); i != end; ++i)
    {
        file_status status = i->status();
        if (is_regular(status))
        {
            load(i->path().string());
        } else if (is_directory(status) && (*i->path().leaf().begin()) != '.')
        {
            loadDir(i->path().string());
        }
    }
}

void Gameplay::Playbook::load(const std::string &path)
{
    const char *cpath = path.c_str();
    printf("Playbook: loading %s\n", cpath);
    try
    {
        Play *play = new Play(_gameplay, cpath);
        _plays.push_back(play);
    } catch (ANTLRException &e)
    {
        printf("Error: %s\n", e.toString().c_str());
    }
}

bool Gameplay::Playbook::setup()
{
    if (_goalie)
    {
#if 0
        //FIXME - The goalie can only be changed under certain circumstances.
        if (_goalie->assigned() && !_goalie->robot()->visible())
        {
            // Lost the goalie
            _goalie->robot(0);
        }
#endif
        
        // Assign a goalie if we don't have one.
        if (!_goalie->assigned())
        {
            // Try to assign a robot to be the goalie
            Robot *robot = _goalie->selectRobot();
            if (robot)
            {
                // We have a new goalie
                printf("Robot %d is the goalie\n", robot->id());
                _goalie->robot(robot);
                robot->goalie(true);
            }
        }
    }
    
    // Select a new play if we need one.
    if (!_currentPlay || _currentPlay->done())
    {
        // Log why the play ended, with statistics.
        if (_currentPlay && !_logged_end)
        {
            // Don't do this again until this or another play is started.
            _logged_end = true;
            
            printf("Play \"%s\" ended: ", _currentPlay->name().c_str());
            switch (_currentPlay->status())
            {
                case Play::Timeout:
                    printf("timeout %d/%d\n", _currentPlay->numTimeouts(), _currentPlay->numStarted());
                    break;
                
                case Play::Terminated:
                    printf("terminated %d/%d\n", _currentPlay->numTerminated(), _currentPlay->numStarted());
                    break;
                
                case Play::Completed:
                    printf("completed %d/%d\n", _currentPlay->numCompleted(), _currentPlay->numStarted());
                    break;
                
                case Play::Aborted:
                    printf("aborted %d/%d\n", _currentPlay->numAborted(), _currentPlay->numStarted());
                    break;
                
                default:
                    printf("BAD STATUS\n");
                    break;
            }
        }
        
        // Switch to a new play if it is different from the current play or
        // if it is the same and the current play wants to be restarted.
        Play *next = selectPlay();
        if (next && (next != _currentPlay || (_currentPlay && _currentPlay->restart)))
        {
            // Call stop() here to update statistics.
            if (_currentPlay)
            {
                _currentPlay->stop();
            }
            
            _currentPlay = next;
            _needStart = true;
            
            printf("Switching to play \"%s\"\n", _currentPlay->name().c_str());
            
            _logged_end = false;
        }
    }
    
    return _needStart;
}

void Gameplay::Playbook::run()
{
    // Start the play before running the goalie in case the goalie
    // needs to work with other robots.
    if (_currentPlay)
    {
        if (_needStart)
        {
            _needStart = false;
            _currentPlay->start();
        }
    }
    
    // Run the goalie behavior.
    if (_goalie && _goalie->assigned())
    {
        _goalie->run();
    }
    
    // Run the current play.
    if (_currentPlay)
    {
        _currentPlay->run();
    }
}

void Gameplay::Playbook::abort()
{
    if (_currentPlay)
    {
        if (!_needStart)
        {
            _currentPlay->stop();
        }
        _currentPlay = 0;
    }
    _needStart = false;
}

// Returns true if p1 is better than p2.
bool Gameplay::Playbook::comparePlays(Play *p1, Play *p2)
{
    int availableRobots = _gameplay->availableRobots();
    
    // Prefer roles for which we have enough robots.
    int n1 = p1->num_roles();
    int n2 = p2->num_roles();
    if (availableRobots < n1 && availableRobots >= n2)
    {
        // p1 needs too many robots.
        return false;
    } else if (availableRobots >= n1 && availableRobots < n2)
    {
        // p2 needs too many robots.
        return true;
    }
    
    // Both plays have enough robots.
    
    if (p1->preferenceValue() > p2->preferenceValue())
    {
        return true;
    } else if (p1->preferenceValue() < p2->preferenceValue())
    {
        return false;
    } else {
        //FIXME - Should this move into preferences too?
        // Prefer untested plays.
        if (p1->numStarted() == 0 && p2->numStarted() > 0)
        {
            // p1 hasn't been run but p2 has.
            return true;
        }
        
        if (p2->numStarted() == 0)
        {
            // p2 hasn't been run but p1 has.
            return false;
        }
    }
    
    // Equally good
    return false;
}

void print_plays(const char *label, const vector<Gameplay::Play *> &plays)
{
    printf("%s:\n", label);
    BOOST_FOREACH(Gameplay::Play *play, plays)
    {
        printf("    %p: %s\n", play, play->name().c_str());
    }
}

Gameplay::Play *Gameplay::Playbook::selectPlay()
{
    int availableRobots = _gameplay->availableRobots();
    
    // Subtract one for the goalie.
    if (availableRobots && _goalie && _goalie->assigned())
    {
        --availableRobots;
    }

    // Can't select a play with no robots.
    if (availableRobots == 0)
    {
        return 0;
    }

    // Build a vector of all plays that can work in the current situation.
    // This must be a vector for sort.
    vector<Play *> applicable;
    BOOST_FOREACH(Play *play, _plays)
    {
        if (play->require.all())
        {
            applicable.push_back(play);
            play->updatePreference();
        }
    }
    
    // NOTE: This is unintuitive, but correct based on how I have defined things:
    //       If you have two equally good non-restarting plays, they will appear
    //       to alternate even though plays should be chosen randomly.
    //       This happens because the first play finishes and a new play is
    //       randomly selected each frame.  If the first play is reselected,
    //       nothing happens because it is current and non-restarting.
    //       The reselection continues each frame until the second play is selected.
    //       After the second play finishes reselection happens again until
    //       the first play is selected.
    //
    //       The notable result is that a non-restarting play can run for a
    //       random number of frames after it has finished if it is just as good
    //       as other plays.
    
    // Randomize first, so plays with the same score will be chosen randomly.
    random_shuffle(applicable.begin(), applicable.end());
//    print_plays("Random", applicable);
    
    // Sort by score, with untested plays at the top.
    stable_sort(applicable.begin(), applicable.end(), bind(&Playbook::comparePlays, this, _1, _2));
//    print_plays("Sorted", applicable);
    
    if (!applicable.empty())
    {
        return applicable.front();
    } else {
        return 0;
    }
}

void Gameplay::Playbook::goalie(Behavior *goalie)
{
    Robot *robot = 0;
    if (_goalie)
    {
        robot = _goalie->robot();
        _goalie->robot(0);
    }
    
    _goalie = goalie;
    
    // If we are changing goalie behaviors, keep the same robot.
    if (_goalie && robot)
    {
        // This will start the goalie.
        _goalie->robot(robot);
    }
}
